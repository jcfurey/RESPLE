#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <boost/make_shared.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/empty.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "livox_interfaces/msg/custom_msg.hpp"
#include "livox_ros_driver/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "estimate_msgs/msg/calib.hpp"
#include "estimate_msgs/msg/spline.hpp"
#include "estimate_msgs/msg/estimate.hpp"
#include "estimate_msgs/action/save_map.hpp"
#include "Estimator.h"

KD_TREE<pcl::PointXYZINormal> ikdtree;

class RESPLE : public rclcpp_lifecycle::LifecycleNode
{

public:
    explicit RESPLE(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp_lifecycle::LifecycleNode("RESPLE", 
          rclcpp::NodeOptions(options).use_intra_process_comms(true)),
          diagnostics_(this),
          processing_active_(false)
    {
        RCLCPP_INFO(this->get_logger(), "RESPLE LifecycleNode created (unconfigured state)");
    }
    
    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring RESPLE...");
        lidar_to_baselink_ = Eigen::Affine3d::Identity();
        imu_to_baselink_ = geometry_msgs::msg::TransformStamped();
        
        // Parameter validation with constraints
        auto num_threads_desc = rcl_interfaces::msg::ParameterDescriptor{};
        num_threads_desc.description = "Number of OpenMP threads for parallel processing";
        num_threads_desc.integer_range.resize(1);
        num_threads_desc.integer_range[0].from_value = 1;
        num_threads_desc.integer_range[0].to_value = 16;
        num_threads_desc.integer_range[0].step = 1;
        num_threads_ = this->declare_parameter<int>("num_threads", 5, num_threads_desc);
        
        auto num_match_points_desc = rcl_interfaces::msg::ParameterDescriptor{};
        num_match_points_desc.description = "Number of nearest neighbor points for matching";
        num_match_points_desc.integer_range.resize(1);
        num_match_points_desc.integer_range[0].from_value = 3;
        num_match_points_desc.integer_range[0].to_value = 10;
        num_match_points_desc.integer_range[0].step = 1;
        num_match_points_ = this->declare_parameter<int>("num_match_points", 5, num_match_points_desc);
        
        RCLCPP_INFO(this->get_logger(), "Using %d threads for parallel processing", num_threads_);
        RCLCPP_INFO(this->get_logger(), "Using %d nearest neighbor points for matching", num_match_points_);
        
        // Setup diagnostics
        diagnostics_.setHardwareID("RESPLE");
        diagnostics_.add("System Health", this, &RESPLE::updateDiagnostics);
        
        // Initialize diagnostic metrics
        last_process_time_ = this->now();
        frame_count_ = 0;
        total_computation_time_ms_ = 0.0;
        total_iekf_iterations_ = 0;
        
        readParameters();
        
        // Create callback groups to separate sensor IO from control/estimation callbacks
        sensor_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // Create publishers (inactive until activated)
        pub_est = this->create_publisher<estimate_msgs::msg::Estimate>("est_window", rclcpp::QoS(50).reliable());
        pub_start_time = this->create_publisher<std_msgs::msg::Int64>("start_time", rclcpp::QoS(50).reliable());
        // Use transient_local durability for pose to match Nav2 expectations and ensure late subscribers get last pose
        pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose", rclcpp::QoS(1).transient_local().reliable());
        pub_pose_cov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose_cov", rclcpp::QoS(1).transient_local().reliable());            
        pub_cur_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", rclcpp::QoS(2).reliable());
        br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create SaveMap action server
        save_map_action_server_ = rclcpp_action::create_server<estimate_msgs::action::SaveMap>(
            this,
            "save_map",
            std::bind(&RESPLE::handleSaveMapGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RESPLE::handleSaveMapCancel, this, std::placeholders::_1),
            std::bind(&RESPLE::handleSaveMapAccepted, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "RESPLE configured successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Activating RESPLE...");
        
        // Activate publishers
        pub_est->on_activate();
        pub_start_time->on_activate();
        pub_pose->on_activate();
        pub_pose_cov->on_activate();
        pub_cur_scan->on_activate();
        
        // Setup subscriptions
        rclcpp::SubscriptionOptions sensor_sub_opt;
        sensor_sub_opt.callback_group = sensor_cb_group;
        auto imu_qos = rclcpp::SensorDataQoS().keep_last(200).best_effort();
        auto lidar_qos = rclcpp::SensorDataQoS().keep_last(100).best_effort();
        
        if (!if_lidar_only) {
            std::string imu_type = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "topic_imu", "imu");
            sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_type, imu_qos, std::bind(&RESPLE::getImuCallback, this, std::placeholders::_1), sensor_sub_opt);
        }
        
        auto lidar_names = this->declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>());
        assert(this->get_parameter("lidars", lidar_names));
        if (lidar_names.empty()) {
            LidarConfig lidar(this->get_node_parameters_interface(), "");
            lidars.emplace(lidar.type, lidar);
            lidars_data.emplace(std::piecewise_construct, std::make_tuple(lidar.type), std::make_tuple());
        } else {
            for (const auto& lidar_name : lidar_names) {
                LidarConfig lidar(this->get_node_parameters_interface(), lidar_name + ".");
                lidars.emplace(lidar.type, lidar);
                lidars_data.emplace(std::piecewise_construct, std::make_tuple(lidar.type), std::make_tuple());
            }
        }
        
        for (const auto& [lidar_name, lidar] : lidars) {
            if (!lidar.type.compare("Ouster")) {
                sub_ouster = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, lidar_qos, std::bind(&RESPLE::ousterLidarCallback<ouster_ros::Point>, this, std::placeholders::_1), sensor_sub_opt);
            } else if (!lidar.type.compare("Mid70Avia")) {
                sub_livox = this->create_subscription<livox_ros_driver::msg::CustomMsg>(
                        lidar.topic, lidar_qos, std::bind(&RESPLE::livoxLidarCallback, this, std::placeholders::_1), sensor_sub_opt);
            } else if (!lidar.type.compare("HAP360")) {
                sub_livox2 = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                        lidar.topic, lidar_qos, std::bind(&RESPLE::livoxLidar2Callback, this, std::placeholders::_1), sensor_sub_opt);
            } else if (!lidar.type.compare("AviaResple")) {
                sub_livox_avia = this->create_subscription<livox_interfaces::msg::CustomMsg>(
                        lidar.topic, lidar_qos, std::bind(&RESPLE::livoxAVIACallback, this, std::placeholders::_1), sensor_sub_opt);
            } else if (!lidar.type.compare("Hesai")) {
                sub_hesai = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, lidar_qos, std::bind(&RESPLE::hesaiLidarCallback, this, std::placeholders::_1), sensor_sub_opt);
            } else if (!lidar.type.compare("Mid360Boxi")) {
                sub_livox_mid360_boxi = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, lidar_qos, std::bind(&RESPLE::livoxMid360BoxiCallback, this, std::placeholders::_1), sensor_sub_opt);
            }
        }
        
        // Start processing thread
        processing_active_ = true;
        processing_thread_ = std::thread(&RESPLE::processData, this);
        
        RCLCPP_INFO(this->get_logger(), "RESPLE activated successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating RESPLE...");
        
        // Stop processing thread
        processing_active_ = false;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        // Deactivate publishers
        pub_est->on_deactivate();
        pub_start_time->on_deactivate();
        pub_pose->on_deactivate();
        pub_pose_cov->on_deactivate();
        pub_cur_scan->on_deactivate();
        
        // Reset subscriptions
        sub_imu.reset();
        sub_ouster.reset();
        sub_livox.reset();
        sub_livox2.reset();
        sub_livox_avia.reset();
        sub_hesai.reset();
        sub_livox_mid360_boxi.reset();
        
        RCLCPP_INFO(this->get_logger(), "RESPLE deactivated successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up RESPLE...");
        
        // Clear buffers and data structures
        lidars.clear();
        lidars_data.clear();
        imu_buff.clear();
        imu_meas.clear();
        pt_meas.clear();
        pc_world.clear();
        accum_nearest_points.clear();
        
        // Reset publishers
        pub_est.reset();
        pub_start_time.reset();
        pub_pose.reset();
        pub_pose_cov.reset();
        pub_cur_scan.reset();
        br.reset();
        
        RCLCPP_INFO(this->get_logger(), "RESPLE cleaned up successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down RESPLE...");
        
        // Ensure processing thread is stopped
        processing_active_ = false;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        RCLCPP_INFO(this->get_logger(), "RESPLE shutdown complete");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void processData()
    {
        rclcpp::Rate rate(20);
        int64_t max_spl_knots = 0;
        int64_t t_last_map_upd = 0;
        while (processing_active_ && rclcpp::ok()) {
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.t_buff.empty()) {
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_frame(new pcl::PointCloud<pcl::PointXYZINormal>());
                    lidar_data.mtx_pc.lock();
                    pc_frame->points = lidar_data.pc_buff.front();
                    lidar_data.pc_buff.pop_front();
                    int64_t time_begin = lidar_data.t_buff.front();
                    lidar_data.t_buff.pop_front();
                    lidar_data.mtx_pc.unlock();
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(*pc_frame, *pc_frame, indices);
                    pc_last_ds->clear();

                    ds_filter_body.setInputCloud(pc_frame);
                    ds_filter_body.filter(*pc_last_ds);
                    sort(pc_last_ds->points.begin(), pc_last_ds->points.end(), &CommonUtils::time_list);
                    const LidarConfig& lidar = lidars.at(lidar_name);
                    for (size_t i = 0; i < pc_last_ds->points.size(); i++) {
                        PointData pt(pc_last_ds->points[i], time_begin, lidar.q_bl, lidar.t_bl, lidar.w_pt);
                        lidar_data.pt_buff.push_back(pt);
                    }
                }
            }            
            if (!if_lidar_only && !imu_int_buff.empty()) {
                m_buff.lock();
                Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_buff_msg = imu_int_buff;
                imu_int_buff.clear();
                m_buff.unlock();
                for (size_t i = 0; i < imu_buff_msg.size(); i++) {
                    const auto imu_msg = imu_buff_msg[i];
                    int64_t t_ns = rclcpp::Time(imu_msg->header.stamp).nanoseconds();
                    Eigen::Vector3d acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                    if (acc_ratio) acc *= 9.81;
                    Eigen::Vector3d gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                    ImuData imu(t_ns, gyro, acc); 
                    imu_buff.push_back(imu);
                }
            }
            if(!initialization()) {
                if (rclcpp::ok()) {
                    rate.sleep();
                }
                continue;
            }
            while (collectMeasurements()) {
                // Track computation time
                auto frame_start = std::chrono::high_resolution_clock::now();
                
                int64_t max_time_ns = pt_meas.back().time_ns;
                if (if_lidar_only) {
                    estimator_lo.propRCP(max_time_ns);
                    estimator_lo.updateIEKFLiDAR(pt_meas, &ikdtree, param.nn_thresh, param.coeff_cov, num_threads_, num_match_points_);
                    total_iekf_iterations_ += estimator_lo.n_iter;
                } else {
                    if (!imu_meas.empty()) {
                        max_time_ns = std::max(imu_meas.back().time_ns, max_time_ns);
                    }
                    while (!imu_meas.empty() && imu_meas.front().time_ns < spline->maxTimeNs() - spline->getKnotTimeIntervalNs()) {
                        imu_meas.pop_front();
                    }                         
                    estimator_lio.propRCP(max_time_ns);
                    estimator_lio.updateIEKFLiDARInertial(pt_meas, &ikdtree, param.nn_thresh, imu_meas, gravity, param.cov_acc, param.cov_gyro, param.coeff_cov, num_threads_, num_match_points_);
                    total_iekf_iterations_ += estimator_lio.n_iter;
                }
                #pragma omp parallel for num_threads(num_threads_)
                for (size_t i = 0; i < pt_meas.size(); i++) {
                    PointData& pt_data = pt_meas[i];            
                    Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w, pt_data.t_bl, pt_data.q_bl);
                }            
                for (size_t i = 0; i < pt_meas.size(); i++) {
                    PointData& pt_data = pt_meas[i];
                    pc_world.points.push_back(pt_data.pt_w);
                    accum_nearest_points.push_back(pt_data.nearest_points);
                }
                pt_meas.clear();
                if (spline->numKnots() > max_spl_knots) {
                    estimate_msgs::msg::Spline spline_msg;
                    spline->getSplineMsg(spline_msg, std::max(int(max_spl_knots-1),0));
                    estimate_msgs::msg::Estimate est_msg;
                    est_msg.spline = spline_msg;
                    est_msg.if_full_window.data = (spline->numKnots() >= 4);
                    est_msg.runtime.data = 0;
                    pub_est->publish(est_msg);  
                    max_spl_knots = spline->numKnots();       

                    // Publish current pose
                    int64_t pose_time_ns = spline->maxTimeNs();
                    Eigen::Vector3d t_pose = spline->itpPosition(pose_time_ns);
                    Eigen::Quaterniond q_pose;
                    spline->itpQuaternion(pose_time_ns, &q_pose);

                    geometry_msgs::msg::PoseStamped pose_msg = 
                        CommonUtils::pose2msg(pose_time_ns, t_pose, q_pose);
                    pose_msg.header.frame_id = odom_id;
                    pub_pose->publish(pose_msg);                        

                    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg = 
                        CommonUtils::pose2msg(pose_time_ns, t_pose, q_pose, cov_pose);
                    pose_cov_msg.header.frame_id = odom_id;
                    pub_pose_cov->publish(pose_cov_msg);
                }
                if (max_time_ns >= t_last_map_upd + 1e8) {
                    mapIncremental();
                    publishFrameWorld();
                    lasermapFovSegment();
                    pc_world.clear();
                    accum_nearest_points.clear();
                    t_last_map_upd = max_time_ns;
                }
                
                // Update diagnostic metrics
                auto frame_end = std::chrono::high_resolution_clock::now();
                auto frame_duration = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
                total_computation_time_ms_ += frame_duration.count() / 1000.0;
                frame_count_++;
                
                // Update diagnostics at 1 Hz
                if ((this->now() - last_process_time_).seconds() >= 1.0) {
                    diagnostics_.force_update();
                    last_process_time_ = this->now();
                }
            }
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // Callback groups (initialized in constructor)
    rclcpp::CallbackGroup::SharedPtr sensor_cb_group;

    // Performance tuning parameters (Phase 3)
    int num_threads_;
    int num_match_points_;
    
    // Diagnostics
    diagnostic_updater::Updater diagnostics_;
    rclcpp::Time last_process_time_;
    size_t frame_count_;
    double total_computation_time_ms_;
    size_t total_iekf_iterations_;
    
    // Lifecycle management
    std::atomic<bool> processing_active_;
    std::thread processing_thread_;
    
    // SaveMap action server
    using SaveMapAction = estimate_msgs::action::SaveMap;
    using GoalHandleSaveMap = rclcpp_action::ServerGoalHandle<SaveMapAction>;
    rclcpp_action::Server<SaveMapAction>::SharedPtr save_map_action_server_;
    
    // Pre-allocated reusable buffers (avoid repeated heap allocations)
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_frame_reusable_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_world_reusable_;

    std::string node_name = "RESPLE";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ouster;
    rclcpp::Subscription<livox_ros_driver::msg::CustomMsg>::SharedPtr sub_livox;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox2;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_livox_avia;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_hesai;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_mid360_boxi;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cur_scan;
    rclcpp_lifecycle::LifecyclePublisher<estimate_msgs::msg::Estimate>::SharedPtr pub_est;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr pub_start_time;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_imu_transform_ = false;
    bool have_lidar_transform_ = false;
    Eigen::Affine3d lidar_to_baselink_;
    geometry_msgs::msg::TransformStamped imu_to_baselink_;
    
    std::string frame_id;
    std::string odom_id;

    std::map<std::string, LidarConfig> lidars;
    float ds_lm_voxel;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_body;    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last_ds;
    pcl::PointCloud<pcl::PointXYZINormal> pc_world;
    int point_filter_num = 1;
    int64_t time_offset = 0;

    std::vector<BoxPointType> cub_needrm;
    BoxPointType LocalMap_Points;
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> accum_nearest_points;
    double cube_len = 2000; 
    const float MOV_THRESHOLD = 1.5f;
    float det_range = 100.0;
    bool if_init_map = false;
    struct LidarData {
        Eigen::aligned_deque<Eigen::aligned_vector<pcl::PointXYZINormal>> pc_buff;
        std::deque<int64_t> t_buff;
        std::mutex mtx_pc;
        Eigen::aligned_deque<PointData> pt_buff;
    };
    std::map<std::string, LidarData> lidars_data;    
    Eigen::aligned_deque<PointData> pt_meas;    

    bool if_lidar_only;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    Eigen::aligned_deque<ImuData> imu_buff;
    Eigen::aligned_deque<ImuData> imu_meas;
    Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_int_buff;    
    std::mutex m_buff;
    bool acc_ratio;
    Eigen::Vector3d cov_ba;
    Eigen::Vector3d cov_bg;
    Eigen::Vector<double, 6> cov_pose;        
    Eigen::Vector3d gravity;

    bool if_init_filter = false;
    Estimator<24> estimator_lo;
    Estimator<30> estimator_lio;
    SplineState* spline;
    double cov_P0 = 0.02;
    double cov_RCP_pos_old = 0.02;
    double cov_RCP_ort_old = 0.02;
    double cov_RCP_pos_new = 0.1;    
    double cov_RCP_ort_new = 0.1;    
    double cov_sys_pos = 0.1;    
    double cov_sys_ort = 0.01;    
    Parameters param;
    int64_t dt_ns;
    int num_points_upd;
    
    // IMU initialization parameters
    int imu_init_num_samples_ = 50;
    double imu_init_max_variance_ = 5.0;

    
    // SaveMap action server handlers
    rclcpp_action::GoalResponse handleSaveMapGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SaveMapAction::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received save map request: %s", goal->filename.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleSaveMapCancel(
        const std::shared_ptr<GoalHandleSaveMap> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel save map");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handleSaveMapAccepted(const std::shared_ptr<GoalHandleSaveMap> goal_handle)
    {
        // Execute in separate thread to not block action server
        std::thread{std::bind(&RESPLE::executeSaveMap, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    void executeSaveMap(const std::shared_ptr<GoalHandleSaveMap> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing save map action...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SaveMapAction::Feedback>();
        auto result = std::make_shared<SaveMapAction::Result>();
        
        try {
            // Get all points from ikd-tree
            feedback->status = "Extracting points from map...";
            feedback->progress = 10.0;
            goal_handle->publish_feedback(feedback);
            
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            map_cloud->points = ikdtree.PCL_Storage;
            map_cloud->width = map_cloud->points.size();
            map_cloud->height = 1;
            map_cloud->is_dense = false;
            
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Save map operation was cancelled";
                result->points_saved = 0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Save map cancelled");
                return;
            }
            
            feedback->status = "Writing map to file...";
            feedback->progress = 50.0;
            goal_handle->publish_feedback(feedback);
            
            // Save to PCD file
            if (pcl::io::savePCDFileBinary(goal->filename, *map_cloud) == -1) {
                result->success = false;
                result->message = "Failed to write PCD file: " + goal->filename;
                result->points_saved = 0;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
                return;
            }
            
            feedback->status = "Map saved successfully";
            feedback->progress = 100.0;
            goal_handle->publish_feedback(feedback);
            
            result->success = true;
            result->message = "Map saved successfully to " + goal->filename;
            result->points_saved = map_cloud->points.size();
            goal_handle->succeed(result);
            
            RCLCPP_INFO(this->get_logger(), "Saved %u points to %s", 
                       result->points_saved, goal->filename.c_str());
                       
        } catch (const std::exception& e) {
            result->success = false;
            result->message = std::string("Exception during save: ") + e.what();
            result->points_saved = 0;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        }
    }

    void readParameters()
    {
        // Frame ID parameters
        frame_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "frame_id", "base_footprint");
        odom_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "odom_frame_id", "odom");
        
        RCLCPP_INFO(this->get_logger(), "Frame IDs - odom: %s, body: %s", 
                    odom_id.c_str(), frame_id.c_str());
        
        ds_lm_voxel = CommonUtils::readParam<float>(this->get_node_parameters_interface(), "ds_lm_voxel", 0.0);
        
        // Validate ds_scan_voxel parameter
        float ds_scan_voxel = CommonUtils::readParam<float>(this->get_node_parameters_interface(), "ds_scan_voxel", 0.0);
        if (ds_scan_voxel < 0.01 || ds_scan_voxel > 1.0) {
            RCLCPP_WARN(this->get_logger(), 
                "ds_scan_voxel value %.3f outside recommended range [0.01, 1.0]", ds_scan_voxel);
        }
        ds_filter_body.setLeafSize(ds_scan_voxel, ds_scan_voxel, ds_scan_voxel);
        
        // Validate nn_thresh parameter
        param.nn_thresh = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "nn_thresh", 0.0);
        if (param.nn_thresh < 0.1 || param.nn_thresh > 5.0) {
            RCLCPP_WARN(this->get_logger(), 
                "nn_thresh value %.3f outside recommended range [0.1, 5.0]", param.nn_thresh);
        }
        if_lidar_only = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "if_lidar_only", false);
        if (!if_lidar_only) {
            acc_ratio = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "acc_ratio", false);
            std::vector<double> bias_acc_var = CommonUtils::readParam<std::vector<double>>(this->get_node_parameters_interface(), "cov_ba", {0.2, 0.2, 0.2});
            cov_ba << bias_acc_var.at(0), bias_acc_var.at(1), bias_acc_var.at(2);
            std::vector<double> bias_gyro_var = CommonUtils::readParam<std::vector<double>>(this->get_node_parameters_interface(), "cov_bg", {0.2, 0.2, 0.2});
            cov_bg << bias_gyro_var.at(0), bias_gyro_var.at(1), bias_gyro_var.at(2);
            std::vector<double> acc_var = CommonUtils::readParam<std::vector<double>>(this->get_node_parameters_interface(), "cov_acc", {1.0, 1.0, 1.0});
            param.cov_acc << acc_var.at(0), acc_var.at(1), acc_var.at(2);
            std::vector<double> gyro_var = CommonUtils::readParam<std::vector<double>>(this->get_node_parameters_interface(), "cov_gyro", {0.1, 0.1, 0.1});
            param.cov_gyro << gyro_var.at(0), gyro_var.at(1), gyro_var.at(2);
        }

        dt_ns = 1e9 / CommonUtils::readParam<int>(this->get_node_parameters_interface(), "knot_hz", 100);
        double dt_s = double(dt_ns) * 1e-9;
        cov_P0 = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "cov_P0", 0.02);
        cov_P0 *= (dt_s*dt_s);
        cov_RCP_pos_old = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "cov_RCP_pos_old", 0.5);
        cov_RCP_ort_old = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "cov_RCP_ort_old", 0.5);
        cov_RCP_pos_new = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "cov_RCP_pos_new", 1.0);
        cov_RCP_ort_new = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "cov_RCP_ort_new", 1.0);
        double std_pos = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "std_sys_pos", 0.1);
        double std_ort = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "std_sys_ort", 0.1);
        cov_sys_pos = std_pos*std_pos*dt_s*dt_s;
        cov_sys_ort = std_ort*std_ort*dt_s*dt_s;
        param.coeff_cov = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "coeff_cov", 10);

        cube_len = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "cube_len", 1000.0);
        point_filter_num = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "point_filter_num", 1);
        num_points_upd = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "num_points_upd", 100);
        if (if_lidar_only) {
            estimator_lo.n_iter = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "n_iter", 1);
        } else {
            estimator_lio.n_iter = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "n_iter", 1);
        }
        std::vector<double> cov_var = CommonUtils::readParam<std::vector<double>>(this->get_node_parameters_interface(), "cov_pose", {0.2, 0.2, 0.2, 0.1, 0.1, 0.1});
        cov_pose << cov_var.at(0), cov_var.at(1), cov_var.at(2), cov_var.at(3), cov_var.at(4), cov_var.at(5);

        pc_last.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        pc_last_ds.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        // num_match_points_ is now initialized in constructor from parameters
        
        // Initialize reusable buffers (Phase 3)
        pc_frame_reusable_.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        laser_cloud_world_reusable_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        double lidar_time_offset = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "lidar_time_offset", 0.0);
        time_offset = 1e9*lidar_time_offset;
        
        // Read IMU initialization parameters
        imu_init_num_samples_ = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "imu_init_num_samples", 50);
        imu_init_max_variance_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "imu_init_max_variance", 5.0);
        if (imu_init_num_samples_ < 10) {
            RCLCPP_WARN(this->get_logger(),
                "imu_init_num_samples value %d is very low, using minimum of 10",
                imu_init_num_samples_);
            imu_init_num_samples_ = 10;
        }
        RCLCPP_INFO(this->get_logger(),
            "IMU initialization: samples=%d, max_variance=%.2f",
            imu_init_num_samples_, imu_init_max_variance_);
    }

    void initFilter(int64_t start_t_ns, Eigen::Vector3d t_init = Eigen::Vector3d::Zero(), Eigen::Quaterniond q_init = Eigen::Quaterniond::Identity())
    {
        Eigen::Matrix<double, 24, 24> cov_RCPs = cov_P0 * Eigen::Matrix<double, 24, 24>::Identity();
        Eigen::Matrix<double, 30, 30> Q = Eigen::Matrix<double, 30, 30>::Zero();
        Eigen::Matrix<double, 6, 6> Q_block_old = Eigen::Matrix<double, 6, 6>::Zero();
        Q_block_old.topLeftCorner<3, 3>() = cov_RCP_pos_old*cov_sys_pos *Eigen::Matrix3d::Identity();
        Q_block_old.bottomRightCorner<3, 3>() = cov_RCP_ort_old*cov_sys_ort *Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> Q_block_new = Eigen::Matrix<double, 6, 6>::Zero();
        Q_block_new.topLeftCorner<3, 3>() = cov_RCP_pos_new*cov_sys_pos *Eigen::Matrix3d::Identity();
        Q_block_new.bottomRightCorner<3, 3>() = cov_RCP_ort_new*cov_sys_ort *Eigen::Matrix3d::Identity();
        Q.topLeftCorner<6, 6>() = Q_block_old;
        Q.block<6, 6>(6, 6) = Q_block_old;
        Q.block<6, 6>(12, 12) = Q_block_old;
        Q.bottomRightCorner<6, 6>() = Q_block_new;
        if (if_lidar_only) {
            estimator_lo.setState(dt_ns, start_t_ns, t_init, q_init, Q.topLeftCorner<24, 24>(), cov_RCPs);
            spline = estimator_lo.getSpline();
        } else {
            Eigen::Matrix<double, 30, 30> cov_x = Eigen::Matrix<double, 30, 30>::Zero();
            cov_x.topLeftCorner<24, 24>() = cov_RCPs;
            cov_x.block<3, 3>(24, 24) = cov_ba.asDiagonal();
            cov_x.block<3, 3>(27, 27) = cov_bg.asDiagonal();
            estimator_lio.setState(dt_ns, start_t_ns, t_init, q_init, Q, cov_x);
            spline = estimator_lio.getSpline();
        }
    }

    bool updateImuTransform(std::string source_frame_id)
    {
        if (!have_imu_transform_) {
            try {
                geometry_msgs::msg::TransformStamped transform; 
                if (tf_buffer_->canTransform(this->frame_id, source_frame_id, 
                                                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
                        transform = tf_buffer_->lookupTransform(this->frame_id, source_frame_id, 
                                                                        rclcpp::Time(0));
                        imu_to_baselink_ = transform;
                        
                        have_imu_transform_ = true;
                        RCLCPP_INFO(this->get_logger(), "[RESPLE] Got IMU transform: %s -> %s", 
                                source_frame_id.c_str(), this->frame_id.c_str());
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                            "[RESPLE] Waiting for IMU transform: %s -> %s", 
                                            source_frame_id.c_str(), this->frame_id.c_str());
                        return false;
                    }
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                    "[RESPLE] IMU transform exception: %s", ex.what());
                return false;
            }
        }

        return true;
    }

    bool updateLidarTransform(std::string source_frame_id)
    {
        if (!have_lidar_transform_) {
            try {
                geometry_msgs::msg::TransformStamped transform; 
                if (tf_buffer_->canTransform(this->frame_id, source_frame_id, 
                                                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
                        transform = tf_buffer_->lookupTransform(this->frame_id, source_frame_id, 
                                                                        rclcpp::Time(0));
                        lidar_to_baselink_ = tf2::transformToEigen(transform);
                        
                        have_lidar_transform_ = true;
                        RCLCPP_INFO(this->get_logger(), "[RESPLE] Got LiDAR transform: %s -> %s", 
                                source_frame_id.c_str(), this->frame_id.c_str());
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                            "[RESPLE] Waiting for LiDAR transform: %s -> %s", 
                                            source_frame_id.c_str(), this->frame_id.c_str());
                        return false;
                    }
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                    "[RESPLE] LiDAR transform exception: %s", ex.what());
                return false;
            }
        }

        return true;
    }

    sensor_msgs::msg::Imu::SharedPtr transformImu(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_raw, 
                                                   const geometry_msgs::msg::TransformStamped& transform)
    {
        sensor_msgs::msg::Imu::SharedPtr imu(new sensor_msgs::msg::Imu);
        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);

        // Copy header
        imu->header = imu_raw->header;

        // Transform orientation
        Eigen::Quaterniond orientation(imu_raw->orientation.w, imu_raw->orientation.x, 
                                       imu_raw->orientation.y, imu_raw->orientation.z);
        Eigen::Quaterniond rotation(transform_eigen.rotation());
        Eigen::Quaterniond quat_transformed = orientation * rotation.inverse();

        imu->orientation.w = quat_transformed.w();
        imu->orientation.x = quat_transformed.x();
        imu->orientation.y = quat_transformed.y();
        imu->orientation.z = quat_transformed.z();

        // Transform angular velocity
        Eigen::Vector3d ang_vel(imu_raw->angular_velocity.x,
                                imu_raw->angular_velocity.y,
                                imu_raw->angular_velocity.z);
        Eigen::Vector3d ang_vel_transformed = transform_eigen.rotation() * ang_vel;

        imu->angular_velocity.x = ang_vel_transformed[0];
        imu->angular_velocity.y = ang_vel_transformed[1];
        imu->angular_velocity.z = ang_vel_transformed[2];

        // Transform linear acceleration (accounting for centripetal acceleration)
        Eigen::Vector3d lin_accel(imu_raw->linear_acceleration.x,
                                  imu_raw->linear_acceleration.y,
                                  imu_raw->linear_acceleration.z);
        Eigen::Vector3d lin_accel_transformed = transform_eigen.rotation() * lin_accel
                                               + ang_vel_transformed.cross(ang_vel_transformed.cross(-transform_eigen.translation()));

        imu->linear_acceleration.x = lin_accel_transformed[0];
        imu->linear_acceleration.y = lin_accel_transformed[1];
        imu->linear_acceleration.z = lin_accel_transformed[2];

        return imu;
    }
    
    void getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        // Lookup IMU transform if not yet initialized
        if(!updateImuTransform(imu_msg->header.frame_id)) return;
        
        m_buff.lock();
        if (have_imu_transform_) {
            sensor_msgs::msg::Imu::SharedPtr transformed_imu = transformImu(imu_msg, imu_to_baselink_);
            imu_int_buff.push_back(transformed_imu);
        } else {
            // If no transform available, pass through (assumes IMU already in base_link frame)
            imu_int_buff.push_back(imu_msg);
        }
        m_buff.unlock();
    }
    
    // Diagnostic updater callback
    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
    {
        // Calculate processing rate
        double time_elapsed = (this->now() - last_process_time_).seconds();
        double processing_rate = (time_elapsed > 0) ? frame_count_ / time_elapsed : 0.0;
        double avg_computation_time = (frame_count_ > 0) ? total_computation_time_ms_ / frame_count_ : 0.0;
        double avg_iekf_iters = (frame_count_ > 0) ? static_cast<double>(total_iekf_iterations_) / frame_count_ : 0.0;
        
        // Determine system health
        const double expected_rate = 20.0;  // Target: 20 Hz
        const double warn_threshold = 0.7 * expected_rate;  // 14 Hz
        const double error_threshold = 0.5 * expected_rate;  // 10 Hz
        
        if (frame_count_ == 0) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No frames processed yet");
        } else if (processing_rate < error_threshold) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, 
                        "Processing rate critically low");
        } else if (processing_rate < warn_threshold) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, 
                        "Processing rate below target");
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "System healthy");
        }
        
        // Add detailed metrics
        stat.add("Processing Rate (Hz)", processing_rate);
        stat.add("Target Rate (Hz)", expected_rate);
        stat.add("Frames Processed", static_cast<int>(frame_count_));
        stat.add("Avg Computation Time (ms)", avg_computation_time);
        stat.add("Avg IEKF Iterations", avg_iekf_iters);
        stat.add("Num Threads", num_threads_);
        stat.add("Num Match Points", num_match_points_);
        
        // Buffer sizes
        size_t total_lidar_buffer = 0;
        for (const auto& [name, data] : lidars_data) {
            total_lidar_buffer += data.pc_buff.size();
        }
        stat.add("LiDAR Buffer Size", static_cast<int>(total_lidar_buffer));
        stat.add("IMU Buffer Size", static_cast<int>(imu_buff.size()));
        stat.add("Point Meas Buffer Size", static_cast<int>(pt_meas.size()));
        
        // Reset counters for next period
        frame_count_ = 0;
        total_computation_time_ms_ = 0.0;
        total_iekf_iterations_ = 0;
    }

    template<typename T>
    void ousterLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ouster_msg_in)
    {
        // Find the Ouster lidar config
        std::string name = "Ouster";
        if (lidars.find(name) == lidars.end()) {
            // Try to find any Ouster-type lidar
            for (const auto& [lidar_name, lidar_cfg] : lidars) {
                if (lidar_cfg.type == "Ouster") {
                    name = lidar_name;
                    break;
                }
            }
        }
        const LidarConfig& lidar = lidars.at(name);
        
        // Lookup LiDAR transform
        if(!updateLidarTransform(ouster_msg_in->header.frame_id)) return;
        
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        typename pcl::PointCloud<T>::Ptr pc_last_ouster(new typename pcl::PointCloud<T>());
        pcl::fromROSMsg(*ouster_msg_in, *pc_last_ouster);
        size_t plsize = pc_last_ouster->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds() - time_offset;
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        pcl::PointXYZINormal pt;
        float blind = lidar.blind;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                pt.x = pc_last_ouster->points[i].x;
                pt.y = pc_last_ouster->points[i].y;
                pt.z = pc_last_ouster->points[i].z;
                pt.intensity = float (pc_last_ouster->points[i].t) / float (1e6); // unit: ms
                pt.curvature = pc_last_ouster->points[i].intensity;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && pc_last_ouster->points[i].t + time_begin > last_t_ns) {
                    pc_last->points.push_back(pt);
                    int64_t ofs = pc_last_ouster->points[i].t;
                    max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                }
            }
        }

        // Transform point cloud to body frame
        pcl::transformPointCloud(*pc_last, *pc_last, lidar_to_baselink_);

        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_last->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
    }

    void livoxLidarCallback(const livox_ros_driver::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        std::string name = "Mid70Avia";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        int valid_point_num = 0;
        pcl::PointXYZINormal pt_pre;
        pt_pre.x = livox_msg_in->points[0].x;
        pt_pre.y = livox_msg_in->points[0].y;
        pt_pre.z = livox_msg_in->points[0].z;
        int N_SCAN_LINES = lidar.scan_line;
        float blind = lidar.blind;
        for (int i = 1; i < plsize; ++i) {
            if ((livox_msg_in->points[i].line < N_SCAN_LINES) && ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00)) {
                valid_point_num++;
                if (valid_point_num % point_filter_num == 0) {
                    pcl::PointXYZINormal pt;
                    pt.x = livox_msg_in->points[i].x;
                    pt.y = livox_msg_in->points[i].y;
                    pt.z = livox_msg_in->points[i].z;
                    pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms
                    pt.curvature = livox_msg_in->points[i].reflectivity;
                    if (pt.intensity >= 0 && ((abs(pt.x - pt_pre.x) > 1e-7) || (abs(pt.y - pt_pre.y) > 1e-7) || (abs(pt.z - pt_pre.z) > 1e-7))
                                            && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)&& livox_msg_in->points[i].offset_time + time_begin > last_t_ns) {
                        int64_t ofs = livox_msg_in->points[i].offset_time;
                        max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                        pc_last->points.push_back(pt);
                    }
                    pt_pre = pt;
                }

            }
        }

        // Transform point cloud to body frame
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());      
        pcl::transformPointCloud(*pc_last, *pc_transformed, lidar_to_baselink_);

        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_transformed->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
    }

    void livoxLidar2Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        std::string name = "HAP360";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        int valid_point_num = 0;
        pcl::PointXYZINormal pt_pre;
        pt_pre.x = livox_msg_in->points[0].x;
        pt_pre.y = livox_msg_in->points[0].y;
        pt_pre.z = livox_msg_in->points[0].z;
        int N_SCAN_LINES = lidar.scan_line;
        float blind = lidar.blind;
        for (int i = 1; i < plsize; ++i) {
            if ((livox_msg_in->points[i].line < N_SCAN_LINES) && ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00)) {
                valid_point_num++;
                if (valid_point_num % point_filter_num == 0) {
                    pcl::PointXYZINormal pt;
                    pt.x = livox_msg_in->points[i].x;
                    pt.y = livox_msg_in->points[i].y;
                    pt.z = livox_msg_in->points[i].z;
                    pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6);
                    pt.curvature = livox_msg_in->points[i].reflectivity;
                    if (pt.intensity >= 0 && ((abs(pt.x - pt_pre.x) > 1e-7) || (abs(pt.y - pt_pre.y) > 1e-7) || (abs(pt.z - pt_pre.z) > 1e-7))
                                            && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && livox_msg_in->points[i].offset_time + time_begin > last_t_ns) {
                        int64_t ofs = livox_msg_in->points[i].offset_time;
                        max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                        pc_last->points.push_back(pt);
                    }
                    pt_pre = pt;
                }
            }
        }

        // Transform point cloud to body frame
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());  
        pcl::transformPointCloud(*pc_last, *pc_transformed, lidar_to_baselink_);

        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_transformed->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
    }

     void livoxAVIACallback(const livox_interfaces::msg::CustomMsg::SharedPtr livox_msg_in)
     {
        std::string name = "AviaResple";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        int valid_point_num = 0;
        pcl::PointXYZINormal pt_pre;
        pt_pre.x = livox_msg_in->points[0].x;
        pt_pre.y = livox_msg_in->points[0].y;
        pt_pre.z = livox_msg_in->points[0].z;
        int N_SCAN_LINES = lidar.scan_line;
        float blind = lidar.blind;
        for (int i = 1; i < plsize; ++i) {
            if ((livox_msg_in->points[i].line < N_SCAN_LINES) && ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) && livox_msg_in->points[i].offset_time + time_begin > last_t_ns) {
                valid_point_num++;
                if (valid_point_num % point_filter_num == 0) {
                    pcl::PointXYZINormal pt;
                    pt.x = livox_msg_in->points[i].x;
                    pt.y = livox_msg_in->points[i].y;
                    pt.z = livox_msg_in->points[i].z;
                    pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6);
                    pt.curvature = livox_msg_in->points[i].reflectivity;
                    if (pt.intensity >= 0 && ((abs(pt.x - pt_pre.x) > 1e-7) || (abs(pt.y - pt_pre.y) > 1e-7) ||
                                            (abs(pt.z - pt_pre.z) > 1e-7))
                                            && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)) {
                        int64_t ofs = livox_msg_in->points[i].offset_time;
                        max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                        pc_last->points.push_back(pt);
                    }
                    pt_pre = pt;
                }
            }
        }

        // Transform point cloud to body frame
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>()); 
        pcl::transformPointCloud(*pc_last, *pc_transformed, lidar_to_baselink_);

        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_transformed->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
     }

    void hesaiLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr hesai_msg_in)
	{
        std::string name = "Hesai";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(hesai_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::PointCloud<hesai_ros::Point>::Ptr pc_last_hesai(new pcl::PointCloud<hesai_ros::Point>());
        pcl::fromROSMsg(*hesai_msg_in, *pc_last_hesai);
        size_t plsize = pc_last_hesai->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(hesai_msg_in->header.stamp);
        int64_t time_begin = timestamp_begin.nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        pcl::PointXYZINormal pt;
        float blind = lidar.blind;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                pt.x = pc_last_hesai->points[i].x;
                pt.y = pc_last_hesai->points[i].y;
                pt.z = pc_last_hesai->points[i].z;
                double timestamp_s;
                double timestamp_ns = std::modf(pc_last_hesai->points[i].timestamp, &timestamp_s);
                rclcpp::Time timestamp_ros(static_cast<int32_t>(timestamp_s), static_cast<int32_t>(timestamp_ns * 1.0e9),
                    rcl_clock_type_t::RCL_ROS_TIME);
                pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3;
                pt.curvature = pc_last_hesai->points[i].intensity;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && CommonUtils::ms2ns(pt.intensity) + time_begin > last_t_ns) {
                    int64_t ofs = CommonUtils::ms2ns(pt.intensity);
                    max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                    pc_last->points.push_back(pt);
                }
            }
        }

        // Transform point cloud to body frame
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());      
        pcl::transformPointCloud(*pc_last, *pc_transformed, lidar_to_baselink_);

        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_transformed->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
	}

    void livoxMid360BoxiCallback(const sensor_msgs::msg::PointCloud2::SharedPtr livox_msg_in)
	{
        std::string name = "Mid360Boxi";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::PointCloud<livox_mid360_boxi::Point>::Ptr pc_last_livox(new pcl::PointCloud<livox_mid360_boxi::Point>());
        pcl::fromROSMsg(*livox_msg_in, *pc_last_livox);
        size_t plsize = pc_last_livox->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(livox_msg_in->header.stamp);
        int64_t time_begin = timestamp_begin.nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        pcl::PointXYZINormal pt;
        float blind = lidar.blind;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                pt.x = pc_last_livox->points[i].x;
                pt.y = pc_last_livox->points[i].y;
                pt.z = pc_last_livox->points[i].z;
                rclcpp::Time timestamp_ros(static_cast<int64_t>(pc_last_livox->points[i].timestamp),
                    rcl_clock_type_t::RCL_ROS_TIME);
                pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3;
                pt.curvature = pc_last_livox->points[i].intensity;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && CommonUtils::ms2ns(pt.intensity) + time_begin > last_t_ns) {
                    int64_t ofs = CommonUtils::ms2ns(pt.intensity);
                    max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                    pc_last->points.push_back(pt);
                }
            }
        }

        // Transform point cloud to body frame
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());    
        pcl::transformPointCloud(*pc_last, *pc_transformed, lidar_to_baselink_);

        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_transformed->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
	}

    void publishFrameWorld()
    {
        int size = pc_world.points.size();
        laser_cloud_world_reusable_->clear();
        laser_cloud_world_reusable_->points.resize(size);
        for (int i = 0; i < size; i++) {
            laser_cloud_world_reusable_->points[i].x = pc_world.points[i].x;
            laser_cloud_world_reusable_->points[i].y = pc_world.points[i].y;
            laser_cloud_world_reusable_->points[i].z = pc_world.points[i].z;
            laser_cloud_world_reusable_->points[i].intensity = pc_world.points[i].curvature;
        }
        
        // Standard publishing (compatible with all RMW implementations)
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*laser_cloud_world_reusable_, cloud_msg);
        cloud_msg.header.stamp = rclcpp::Time(spline->maxTimeNs());
        cloud_msg.header.frame_id = odom_id;
        pub_cur_scan->publish(cloud_msg);
    }

    bool initialization()
    {
        if (if_init_filter && if_init_map) {
            return true;
        }
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            if (lidar_data.pt_buff.empty()) {
                return false;
            }
        }
        int64_t start_t_ns = std::numeric_limits<int64_t>::max();
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            start_t_ns = std::min(start_t_ns, std::max(lidar_data.pt_buff.front().time_ns, int64_t(0)));
        }
        if (!if_init_filter) {
            Eigen::Quaterniond q_WI = Eigen::Quaterniond::Identity();
            if (!if_lidar_only) {
                m_buff.lock();
                int buff_size = imu_buff.size();
                
                // Wait for sufficient IMU samples
                if (buff_size < imu_init_num_samples_) {
                    m_buff.unlock();
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Waiting for %d IMU samples for initialization (current: %d)",
                        imu_init_num_samples_, buff_size);
                    return false;
                }
                
                // Compute mean and variance of accelerometer readings
                Eigen::Vector3d gravity_sum(0, 0, 0);
                int n_imu = std::min(imu_init_num_samples_, buff_size);
                for (int i = 0; i < n_imu; i++) {
                    gravity_sum += imu_buff.at(i).accel;
                }
                Eigen::Vector3d gravity_mean = gravity_sum / n_imu;
                
                // Check variance to ensure IMU is stationary
                double accel_variance = 0.0;
                for (int i = 0; i < n_imu; i++) {
                    Eigen::Vector3d diff = imu_buff.at(i).accel - gravity_mean;
                    accel_variance += diff.squaredNorm();
                }
                accel_variance /= n_imu;
                
                if (accel_variance > imu_init_max_variance_) {
                    m_buff.unlock();
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "IMU readings too noisy for initialization (variance: %.4f > %.4f). "
                        "Ensure robot is stationary or increase imu_init_max_variance parameter.",
                        accel_variance, imu_init_max_variance_);
                    return false;
                }
                
                // Clean up old IMU data
                while (!imu_buff.empty() && imu_buff.front().time_ns < start_t_ns) {
                    imu_buff.pop_front();
                }
                m_buff.unlock();
                
                // Initialize orientation from gravity
                Eigen::Vector3d gravity_ave = gravity_mean.normalized() * 9.81;
                Eigen::Matrix3d R0 = CommonUtils::g2R(gravity_ave);
                double yaw = CommonUtils::R2ypr(R0).x();
                R0 = CommonUtils::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
                Eigen::Quaterniond q0(R0);
                q_WI = Quater::positify(q0);
                gravity = q_WI * gravity_ave;
                
                RCLCPP_INFO(this->get_logger(),
                    "IMU initialization successful (samples: %d, variance: %.4f)",
                    n_imu, accel_variance);
            }
            initFilter(start_t_ns, Eigen::Vector3d(0, 0, 0), q_WI);
            if_init_filter = true;
            std_msgs::msg::Int64 start_time;
            start_time.data = start_t_ns;
            pub_start_time->publish(start_time);
        }
        if (!if_init_map) {
            if(ikdtree.Root_Node == nullptr) {
                ikdtree.set_downsample_param(ds_lm_voxel);
            }
            if (if_lidar_only) {
                estimator_lo.propRCP(start_t_ns);
            } else {
                estimator_lio.propRCP(start_t_ns);
            }
            int feats_down_size = 0;
            for (const auto& [lidar_name, lidar_data] : lidars_data) {
                for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + 1e8) {
                        feats_down_size++;
                    } else {
                        break;
                    }
                }
            }
            if(feats_down_size < 100) {
                return false;
            }
            pc_world.clear();
            pc_world.resize(feats_down_size);
            int world_i = 0;
            for (const auto& [lidar_name, lidar_data] : lidars_data) {
                for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + 1e8) {
                        Association::pointBodyToWorld(start_t_ns, spline, lidar_data.pt_buff[i].pt,
                            pc_world.points[world_i], lidar_data.pt_buff[i].t_bl, lidar_data.pt_buff[i].q_bl);
                        world_i++;
                    } else {
                        break;
                    }
                }
            }
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns < start_t_ns + 1e8) {
                    lidar_data.pt_buff.pop_front();
                }
            }
            ikdtree.Build(pc_world.points);
            pc_world.clear();
            if_init_map = true;
        }
        return false;
    }

    bool collectMeasurements()
    {
        rclcpp::Rate rate(20);
        int64_t pt_min_time = std::numeric_limits<int64_t>::max();
        int64_t pt_max_time = std::numeric_limits<int64_t>::max();
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            if (lidar_data.pt_buff.empty()) {
                rate.sleep();
                return false;
            }
            pt_min_time = std::min(pt_min_time, lidar_data.pt_buff.front().time_ns);
            pt_max_time = std::min(pt_max_time, lidar_data.pt_buff.back().time_ns);
        }
        if (pt_max_time <= spline->maxTimeNs() + dt_ns) {
            rate.sleep();
            return false;
        }
        if (!if_lidar_only && (imu_buff.empty() || imu_buff.back().time_ns <= spline->maxTimeNs())) {
            rate.sleep();
            return false;
        }
        int64_t max_time_ns = std::min(spline->maxTimeNs(), pt_min_time + dt_ns);
        if (pt_min_time > max_time_ns) {
            if (if_lidar_only) {
                estimator_lo.propRCP(pt_min_time);
            } else {
                estimator_lio.propRCP(pt_min_time);
            }
            max_time_ns = spline->maxTimeNs();
        }
        if (spline->numKnots() > 4) {
            max_time_ns = spline->maxTimeNs();
        }
        int cnt = 0;
        for (auto& [lidar_name, lidar_data] : lidars_data) {
            while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns <= max_time_ns &&
                    cnt < num_points_upd) {
                if (spline->numKnots() < 10 || lidar_data.pt_buff.front().time_ns >= spline->maxTimeNs() - dt_ns) {
                    pt_meas.emplace_back(lidar_data.pt_buff.front());
                }
                lidar_data.pt_buff.pop_front();
                cnt++;
            }
        }
        if (!if_lidar_only) {
            while (!imu_buff.empty() && imu_buff.front().time_ns < spline->minTimeNs()) {
                imu_buff.pop_front();
            }
            while (!imu_buff.empty() && imu_buff.front().time_ns <= max_time_ns) {
                imu_meas.emplace_back(imu_buff.front());
                imu_buff.pop_front();
            }
        }
        return true;

    }

    Eigen::Vector3d getPositionLiDAR(int64_t t_ns, const Eigen::Vector3d& t_bl)
    {
        if (if_lidar_only) {
            estimator_lo.propRCP(t_ns);
        } else {
            estimator_lio.propRCP(t_ns);
        }
        Eigen::Quaterniond orient_interp;
        Eigen::Vector3d t_interp = spline->itpPosition(t_ns);
        spline->itpQuaternion(t_ns, &orient_interp);
        Eigen::Vector3d t = orient_interp * t_bl + t_interp;
        return t;
    }

    void lasermapFovSegment()
    {
        static bool Localmap_Initialized = false;
        cub_needrm.shrink_to_fit();
        Eigen::Vector3d pos_lidar_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        Eigen::Vector3d pos_lidar_max(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(),
                std::numeric_limits<double>::min());
        for (const auto& [lidar_name, lidar] : lidars) {
            Eigen::Vector3d pos_lidar = getPositionLiDAR(spline->maxTimeNs(), lidar.t_bl);
            pos_lidar_min = pos_lidar_min.array().min(pos_lidar.array()).matrix();
            pos_lidar_max = pos_lidar_max.array().max(pos_lidar.array()).matrix();
        }
        if (!Localmap_Initialized){
            for (int i = 0; i < 3; i++){
                LocalMap_Points.vertex_min[i] = pos_lidar_min(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_lidar_max(i) + cube_len / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++){
            dist_to_map_edge[i][0] = fabs(pos_lidar_min(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_lidar_max(i) - LocalMap_Points.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range || dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range) need_move = true;
        }
        if (!need_move) return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * det_range) * 0.5 * 0.9, double(det_range * (MOV_THRESHOLD -1)));
        for (int i = 0; i < 3; i++){
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range){
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.emplace_back(tmp_boxpoints);
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range){
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.emplace_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        if(cub_needrm.size() > 0) {
            ikdtree.Delete_Point_Boxes(cub_needrm);
        }
    }

    void mapIncremental()
    {
        Eigen::aligned_vector<pcl::PointXYZINormal> PointToAdd;
        Eigen::aligned_vector<pcl::PointXYZINormal> PointNoNeedDownsample;
        int feats_down_size = pc_world.points.size();
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for(int i = 0; i < feats_down_size; i++) {
            const pcl::PointXYZINormal& point = pc_world.points[i];
            if (!accum_nearest_points[i].empty()) {
                const Eigen::aligned_vector<pcl::PointXYZINormal> &points_near = accum_nearest_points[i];
                bool need_add = true;
                pcl::PointXYZINormal downsample_result, mid_point;

                mid_point.x = floor(point.x/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.y = floor(point.y/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.z = floor(point.z/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                if (fabs(points_near[0].x - mid_point.x) > 0.866 * ds_lm_voxel || fabs(points_near[0].y - mid_point.y) > 0.866 * ds_lm_voxel || fabs(points_near[0].z - mid_point.z) > 0.866 * ds_lm_voxel){
                    PointNoNeedDownsample.emplace_back(pc_world.points[i]);
                    continue;
                }
                for (size_t readd_i = 0; readd_i < points_near.size(); readd_i ++) {
                    if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * ds_lm_voxel && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * ds_lm_voxel && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * ds_lm_voxel) {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) PointToAdd.emplace_back(point);
            } else {
                PointNoNeedDownsample.emplace_back(point);
            }
        }
        ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // Phase 4: Lifecycle node initialization
    rclcpp::NodeOptions options;
    auto node = std::make_shared<RESPLE>(options);
    RCLCPP_INFO_STREAM(node->get_logger(), "RESPLE LifecycleNode created");

    // Transition to configured state
    node->configure();
    RCLCPP_INFO_STREAM(node->get_logger(), "RESPLE configured");
    
    // Transition to active state (starts processing)
    node->activate();
    RCLCPP_INFO_STREAM(node->get_logger(), "RESPLE activated - processing started");

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();

    // Cleanup on shutdown (only if context still valid)
    if (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Gracefully shutting down RESPLE...");
        node->deactivate();
        node->cleanup();
        node->shutdown();
    } else {
        // Context already shut down (Ctrl+C), just stop processing
        RCLCPP_WARN(node->get_logger(), "Context invalid, forcing shutdown...");
        // Manually trigger deactivation to stop thread
        node->on_deactivate(node->get_current_state());
    }
    exec.remove_node(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RESPLE)
