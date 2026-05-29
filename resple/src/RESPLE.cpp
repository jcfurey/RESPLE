// 2026-05-01 Eigen aligned-allocator ABI fix — MUST be the first thing in this
// translation unit, before any header that pulls in Eigen (rclcpp + PCL both do
// transitively). See HARDENING.md Phase 1.6 and ikd_Tree.h's matching block for
// the full rationale. Short version: pin EIGEN_MALLOC_ALREADY_ALIGNED=1 so
// Eigen's aligned_malloc/aligned_free both use std::malloc/std::free directly,
// eliminating the handmade-allocator dispatch path that was crashing in
// __libc_free at ikd_Tree.cpp:485.
#ifndef EIGEN_MALLOC_ALREADY_ALIGNED
#define EIGEN_MALLOC_ALREADY_ALIGNED 1
#endif
#ifndef EIGEN_DEFAULT_ALIGN_BYTES
#define EIGEN_DEFAULT_ALIGN_BYTES 16
#endif
#ifndef EIGEN_MAX_ALIGN_BYTES
#define EIGEN_MAX_ALIGN_BYTES 16
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
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
#include <pcl/console/print.h>
#include <queue>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <future>
#include <chrono>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <execinfo.h>
#include <unistd.h>
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
#ifdef RESPLE_USE_CUDA
#include "gpu/cuda_knn.h"
#endif

// Internal linkage: RESPLE.cpp is compiled into both libresple.so and the
// standalone RESPLE executable (the executable re-compiles the source and links
// the lib). With external linkage there were two definitions of `ikdtree`, and
// which one each function bound to depended on ELF symbol interposition. `static`
// gives each translation unit its own instance (matching g_cuda_map below); the
// global is only ever referenced from within this file.
static KD_TREE<pcl::PointXYZINormal> ikdtree;
#ifdef RESPLE_USE_CUDA
static resple_gpu::CudaMap g_cuda_map;
#endif

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
        // Guard direct declare_parameter calls with has_parameter checks so a
        // second on_configure (after on_cleanup) doesn't throw
        // ParameterAlreadyDeclaredException. The other params in
        // readParameters() use CommonUtils::readParam which already does this.
        if (!this->has_parameter("num_threads")) {
            auto num_threads_desc = rcl_interfaces::msg::ParameterDescriptor{};
            num_threads_desc.description = "Number of OpenMP threads for parallel processing";
            num_threads_desc.integer_range.resize(1);
            num_threads_desc.integer_range[0].from_value = 1;
            num_threads_desc.integer_range[0].to_value = 16;
            num_threads_desc.integer_range[0].step = 1;
            num_threads_ = this->declare_parameter<int>("num_threads", 5, num_threads_desc);
        } else {
            num_threads_ = this->get_parameter("num_threads").as_int();
        }

        if (!this->has_parameter("num_match_points")) {
            auto num_match_points_desc = rcl_interfaces::msg::ParameterDescriptor{};
            num_match_points_desc.description = "Number of nearest neighbor points for matching";
            num_match_points_desc.integer_range.resize(1);
            num_match_points_desc.integer_range[0].from_value = 3;
            num_match_points_desc.integer_range[0].to_value = 10;
            num_match_points_desc.integer_range[0].step = 1;
            num_match_points_ = this->declare_parameter<int>("num_match_points", 5, num_match_points_desc);
        } else {
            num_match_points_ = this->get_parameter("num_match_points").as_int();
        }
        
        RCLCPP_INFO(this->get_logger(), "Using %d threads for parallel processing", num_threads_);
        RCLCPP_INFO(this->get_logger(), "Using %d nearest neighbor points for matching", num_match_points_);
        
        // Setup diagnostics
        diagnostics_.setHardwareID("RESPLE");
        diagnostics_.add("System Health", this, &RESPLE::updateDiagnostics);
        
        // Initialize diagnostic metrics
        last_process_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
        frame_count_.store(0, std::memory_order_relaxed);
        total_computation_time_us_.store(0, std::memory_order_relaxed);
        total_iekf_iterations_.store(0, std::memory_order_relaxed);
        
        readParameters();
        
        // Create callback groups to separate sensor IO from control/estimation callbacks
        sensor_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // Create publishers (inactive until activated)
        pub_est = this->create_publisher<estimate_msgs::msg::Estimate>("est_window", rclcpp::QoS(50).reliable());
        // transient_local: start_time is published once after gravity
        // alignment; without late-joiner durability, a Mapping node that
        // subscribes after publish() never sees it and `if_init_succeed`
        // stays false → no path / no odom / Mapping appears dead.
        pub_start_time = this->create_publisher<std_msgs::msg::Int64>(
            "start_time", rclcpp::QoS(1).transient_local().reliable());
        // Use transient_local durability for pose to match Nav2 expectations and ensure late subscribers get last pose
        pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose", rclcpp::QoS(1).transient_local().reliable());
        pub_pose_cov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose_cov", rclcpp::QoS(1).transient_local().reliable());
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(10).reliable());
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

        // Refuse to overwrite a still-joinable worker thread. Lifecycle
        // normally enforces deactivate-before-activate, but a failed
        // transition or external lifecycle client can land us here while the
        // previous processing_thread_ is still alive; the assignment below
        // would invoke std::terminate (move-onto-joinable-thread). Fail the
        // transition cleanly instead.
        if (processing_thread_.joinable()) {
            RCLCPP_ERROR(this->get_logger(),
                "on_activate called while a previous worker thread is still joinable; "
                "refusing to overwrite (would call std::terminate). "
                "Deactivate first.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // Activate publishers
        pub_est->on_activate();
        pub_start_time->on_activate();
        pub_pose->on_activate();
        pub_pose_cov->on_activate();
        pub_odom->on_activate();
        pub_cur_scan->on_activate();
        
        // Setup subscriptions
        rclcpp::SubscriptionOptions sensor_sub_opt;
        sensor_sub_opt.callback_group = sensor_cb_group;
        auto imu_qos = rclcpp::SensorDataQoS().keep_last(200).best_effort();
        auto lidar_qos = rclcpp::SensorDataQoS().keep_last(100).best_effort();
        
        // Always subscribe to IMU for gravity alignment at startup.
        // In LO mode, the subscription is dropped after initialization completes.
        std::string imu_type = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "topic_imu", "imu");
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_type, imu_qos, std::bind(&RESPLE::getImuCallback, this, std::placeholders::_1), sensor_sub_opt);
        
        // Guard with has_parameter so re-activate (after deactivate) doesn't
        // throw ParameterAlreadyDeclaredException → SIGABRT.
        std::vector<std::string> lidar_names;
        if (!this->has_parameter("lidars")) {
            lidar_names = this->declare_parameter<std::vector<std::string>>(
                "lidars", std::vector<std::string>());
        } else {
            lidar_names = this->get_parameter("lidars").as_string_array();
        }
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

        // Stop processing thread with a bounded join so we can't hang the
        // launcher when the worker is wedged inside a third-party lock
        // (ikd-Tree rebuild, CUDA tear-down, FastDDS atexit). The worker
        // loop polls processing_active_ + rclcpp::ok() in 100ms slices
        // (see processData), so under normal conditions the join completes
        // well within the 2s budget.
        processing_active_ = false;
        joinProcessingThreadBounded(std::chrono::seconds(2));
        waitForMapUpdateBounded(std::chrono::seconds(2));

#ifdef RESPLE_USE_CUDA
        // Worker + async map update are now quiesced, so no thread touches
        // g_cuda_map. Drop the GPU-resident map so a re-activation (configure→
        // activate) doesn't run IEKF k-NN against the previous run's stale
        // points before the first mapIncremental re-syncs the GPU. The kd-tree
        // (ikdtree global) is likewise rebuilt on the next init, so both map
        // backends start the new session empty.
        g_cuda_map.clear();
#endif

        // Wait for any in-flight SaveMap action
        {
            std::lock_guard<std::mutex> lock(save_map_mutex_);
            if (save_map_thread_.joinable()) {
                save_map_thread_.join();
            }
        }

        // Deactivate publishers
        pub_est->on_deactivate();
        pub_start_time->on_deactivate();
        pub_pose->on_deactivate();
        pub_pose_cov->on_deactivate();
        pub_odom->on_deactivate();
        pub_cur_scan->on_deactivate();
        
        // Reset subscriptions
        sub_imu.reset();
        sub_ouster.reset();
        sub_livox.reset();
        sub_livox2.reset();
        sub_livox_avia.reset();
        sub_hesai.reset();
        sub_livox_mid360_boxi.reset();

        // Drain any in-flight callbacks before on_cleanup tears down lidars_data
        // / m_buff state. Subscription::reset() drops the application's strong
        // ref but the executor still holds one for any callback already in
        // flight on another thread (sensor_cb_group is MutuallyExclusive, so at
        // most one). Without this barrier, on_cleanup's lidars_data.clear()
        // can deallocate a LidarData while a callback is mid-execution holding
        // a reference to its mtx_pc → SIGSEGV. 100 ms is generous: a single
        // callback completes in ~1 ms on this hardware. This is a temporary
        // fix until rclcpp exposes a callback-group wait_for API in Jazzy.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(this->get_logger(), "RESPLE deactivated successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up RESPLE...");

        // Worker was joined during on_deactivate; bounded wait on the
        // map-update future (the helper checks valid() so this is a no-op
        // when no async map update was ever launched).
        waitForMapUpdateBounded(std::chrono::seconds(2));

        // Clear buffers and data structures
        lidars.clear();
        lidars_data.clear();
        imu_buff.clear();
        imu_meas.clear();
        pt_meas.clear();
        pc_world.clear();
        accum_nearest_points.clear();
        if_init_filter = false;
        if_init_map = false;
        localmap_initialized_ = false;
        imu_first_logged_.store(false);
        imu_count_logged_.store(false);
        lidar_first_logged_.store(false);
        init_complete_logged_.store(false);
        iekf_first_logged_.store(false);
        est_window_first_logged_.store(false);
        
        // Reset publishers
        pub_est.reset();
        pub_start_time.reset();
        pub_pose.reset();
        pub_pose_cov.reset();
        pub_odom.reset();
        pub_cur_scan.reset();
        br.reset();

        // Reset action server: created in on_configure, must be released so a
        // re-configure cycle doesn't orphan the original (and re-creation
        // doesn't conflict on the same goal namespace).
        save_map_action_server_.reset();

        // Reset TF buffer/listener: created in on_configure with the node clock.
        // Holding them across cleanup pins the listener thread.
        tf_listener_.reset();
        tf_buffer_.reset();

        RCLCPP_INFO(this->get_logger(), "RESPLE cleaned up successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down RESPLE...");

        processing_active_ = false;
        // Bounded join + bounded async wait: under SIGINT we have ~5s before
        // the launcher escalates to SIGTERM. If a thread is wedged inside
        // the third-party ikd-Tree, detach instead of hanging the shutdown.
        joinProcessingThreadBounded(std::chrono::seconds(2));
        waitForMapUpdateBounded(std::chrono::seconds(1));

        // Wait for any in-flight SaveMap action
        {
            std::lock_guard<std::mutex> lock(save_map_mutex_);
            if (save_map_thread_.joinable()) {
                save_map_thread_.join();
            }
        }

        RCLCPP_INFO(this->get_logger(), "RESPLE shutdown complete");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Bounded join: dispatch the join() onto an async future and wait
    // with a deadline. Used during lifecycle teardown so that a wedged
    // ikd-Tree (rebuild thread holding internal locks) can't hold up
    // shutdown past the launcher's 5-second SIGINT→SIGTERM grace window.
    void joinProcessingThreadBounded(std::chrono::milliseconds timeout)
    {
        if (!processing_thread_.joinable()) return;
        auto join_done = std::async(std::launch::async,
            [this]{ processing_thread_.join(); });
        if (join_done.wait_for(timeout) != std::future_status::ready) {
            if (processing_thread_.joinable()) {
                // processing_active_ is already false; the worker is just
                // wedged on a third-party lock and will tear down with the
                // process. Detach so shutdown can proceed.
                processing_thread_.detach();
                RCLCPP_WARN(this->get_logger(),
                    "processing thread did not exit within timeout; detached");
            }
        }
    }

    void waitForMapUpdateBounded(std::chrono::milliseconds timeout)
    {
        if (!map_update_future_.valid()) return;
        if (map_update_future_.wait_for(timeout) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(),
                "background map-update did not finish within timeout; "
                "abandoning to avoid blocking shutdown");
            // Move into a thread-local to keep the future alive (and the
            // task running to completion in the background) without making
            // the lifecycle callback wait on it.
            static thread_local std::future<void> abandoned;
            abandoned = std::move(map_update_future_);
        }
    }

    void processData()
    {
        // std::this_thread::sleep_for instead of rclcpp::Rate: rate.sleep()
        // throws runtime_error("context cannot be slept with...") once SIGINT
        // invalidates the rclcpp context, and the existing
        // `if (rclcpp::ok()) rate.sleep()` guards aren't atomic with the
        // sleep itself, so they don't prevent the throw under teardown.
        constexpr auto kRatePeriod = std::chrono::milliseconds(50);  // ~20 Hz
        int64_t max_spl_knots = 0;
        int64_t t_last_map_upd = 0;
        // Heartbeat: every ~2s, dump the worker-thread state so we can
        // distinguish "RESPLE stalled inside a long lock" from "RESPLE
        // is fine but starved of new sensor data" when /est_window goes
        // quiet.
        size_t hb_loop_count = 0;
        size_t hb_iekf_count = 0;
        size_t hb_collect_false_count = 0;
        auto hb_last_log = this->get_clock()->now();
        while (processing_active_ && rclcpp::ok()) {
            // Wrap each iteration in try/catch. Without this, an exception
            // anywhere in the IEKF / collect / deskew / map-update code path
            // unwinds out of the worker thread, terminating it silently. The
            // node then looks "alive but doing nothing": no /odometry
            // publishes, no logs, no crash banner, just stalled. With the
            // wrap, a single bad iteration is logged (throttled, since the
            // root cause likely repeats every cycle) and the worker proceeds
            // to the next scan. This is defense-in-depth — not a replacement
            // for fixing root causes, but ensures the node degrades visibly
            // rather than silently.
            try {
                ++hb_loop_count;
                // Refresh diagnostic buffer-depth caches (read by
                // updateDiagnostics, which can run on the executor thread).
                // pc_buff is callback-written → read under mtx_pc; imu_buff and
                // pt_meas are worker-owned → safe to read here on the worker.
                {
                    int64_t lidar_total = 0;
                    for (auto& [diag_name, diag_data] : lidars_data) {
                        std::lock_guard<std::mutex> lk(diag_data.mtx_pc);
                        lidar_total += static_cast<int64_t>(diag_data.pc_buff.size());
                    }
                    cached_lidar_buf_.store(lidar_total, std::memory_order_relaxed);
                    cached_imu_buf_.store(static_cast<int64_t>(imu_buff.size()), std::memory_order_relaxed);
                    cached_pt_meas_.store(static_cast<int64_t>(pt_meas.size()), std::memory_order_relaxed);
                }
                // Heartbeat dump: read current buffer / spline state every ~2s.
                // Diagnostic only — read-only, no lock-order interactions.
                {
                    auto now_clk = this->get_clock()->now();
                    if ((now_clk - hb_last_log).seconds() >= 2.0) {
                        hb_last_log = now_clk;
                        size_t pc_buff_total = 0, pt_buff_total = 0;
                        for (auto& [n, d] : lidars_data) {
                            pc_buff_total += d.pc_buff.size();
                            pt_buff_total += d.pt_buff.size();
                        }
                        size_t imu_buff_size;
                        {
                            std::lock_guard<std::mutex> lock(m_buff);
                            imu_buff_size = imu_int_buff.size();
                        }
                        const bool init_done = if_init_filter.load();
                        int64_t spline_max = (init_done && spline) ? spline->maxTimeNs() : 0;
                        int64_t spline_n   = (init_done && spline) ? spline->numKnots() : 0;
                        bool map_busy = map_update_pending_.load(std::memory_order_acquire)
                            && map_update_future_.valid()
                            && map_update_future_.wait_for(std::chrono::seconds(0))
                                != std::future_status::ready;
                        RCLCPP_INFO(this->get_logger(),
                            "[RESPLE] heartbeat: loops=%zu iekf=%zu coll_false=%zu "
                            "pc_buff=%zu pt_buff=%zu imu_int=%zu "
                            "spline_knots=%ld spline_max=%ld map_busy=%d",
                            hb_loop_count, hb_iekf_count, hb_collect_false_count,
                            pc_buff_total, pt_buff_total, imu_buff_size,
                            spline_n, spline_max, int(map_busy));
                        hb_loop_count = hb_iekf_count = hb_collect_false_count = 0;
                    }
                }
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.t_buff.empty()) {
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_frame(new pcl::PointCloud<pcl::PointXYZINormal>());
                    int64_t time_begin;
                    {
                        std::lock_guard<std::mutex> lock(lidar_data.mtx_pc);
                        pc_frame->points = lidar_data.pc_buff.front();
                        lidar_data.pc_buff.pop_front();
                        time_begin = lidar_data.t_buff.front();
                        lidar_data.t_buff.pop_front();
                    }
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(*pc_frame, *pc_frame, indices);
                    pc_last_ds->clear();

                    ds_filter_body.setInputCloud(pc_frame);
                    ds_filter_body.filter(*pc_last_ds);
                    sort(pc_last_ds->points.begin(), pc_last_ds->points.end(), &CommonUtils::time_list);
                    const LidarConfig& lidar = lidars.at(lidar_name);
                    for (size_t i = 0; i < pc_last_ds->points.size(); i++) {
                        PointData pt(pc_last_ds->points[i], time_begin, lidar.q_bl, lidar.t_bl, lidar.w_pt, lidar.sensor_origin_body);
                        lidar_data.pt_buff.push_back(pt);
                    }
                }
            }            
            // Drain IMU buffer: always during init (gravity alignment needs it),
            // and during ongoing processing in LIO mode.
            //
            // The empty()-check used to read imu_int_buff without m_buff,
            // racing with getImuCallback's push_back on the deque internals
            // (a real data race that TSan flags). Take m_buff briefly to
            // snapshot the size, then re-acquire it for the swap. Splitting
            // the locks (instead of holding one across the whole block) keeps
            // the IMU callback latency bounded — drains of large buffers don't
            // block the next IMU push.
            bool drain_imu = false;
            {
                std::lock_guard<std::mutex> lock(m_buff);
                drain_imu = (!if_lidar_only.load() || !if_init_filter.load())
                            && !imu_int_buff.empty();
            }
            if (drain_imu) {
                Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_buff_msg;
                {
                    std::lock_guard<std::mutex> lock(m_buff);
                    imu_buff_msg = imu_int_buff;
                    imu_int_buff.clear();
                }
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
                if (if_init_filter && !if_init_map) {
                    // We finished gravity alignment but the map seed isn't
                    // happening — almost always feats_down_size < 100. Log
                    // throttled so the cause is obvious in the launch log.
                    size_t pt_buff_total = 0;
                    for (const auto& [n, d] : lidars_data) pt_buff_total += d.pt_buff.size();
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[RESPLE] init_map blocked: pt_buff total=%zu (need ≥100 in first 100ms)",
                        pt_buff_total);
                }
                std::this_thread::sleep_for(kRatePeriod);
                continue;
            }
            // One-shot: confirm initialization() finally returned true and
            // we entered the steady-state collectMeasurements loop. If this
            // log never fires after gravity alignment, init_map is stuck
            // (typically feats_down_size < 100 or pt_buff drained empty).
            if (!init_complete_logged_.exchange(true)) {
                size_t pt_buff_total = 0;
                for (const auto& [n, d] : lidars_data) pt_buff_total += d.pt_buff.size();
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] initialization complete; entering steady state "
                    "(spline knots=%ld, pt_buff total=%zu)",
                    spline->numKnots(), pt_buff_total);
            }
            bool collected_any = false;
            while (true) {
                bool have_meas;
                {
                    std::lock_guard<std::mutex> spline_lock(spline_mutex_);
                    have_meas = collectMeasurements();
                }
                if (!have_meas) break;
                collected_any = true;
                if (pt_meas.empty() && imu_meas.empty()) { continue; }
                ++hb_iekf_count;
                if (!iekf_first_logged_.exchange(true)) {
                    RCLCPP_INFO(this->get_logger(),
                        "[RESPLE] first IEKF iteration (pt_meas=%zu, spline knots=%ld)",
                        pt_meas.size(), spline->numKnots());
                }
                // Track computation time
                auto frame_start = std::chrono::high_resolution_clock::now();

                int64_t max_time_ns = !pt_meas.empty() ? pt_meas.back().time_ns
                                                        : imu_meas.back().time_ns;
                pt_neighbors_.resize(pt_meas.size());
                {
                    // IEKF reads the map structure (kd-tree or GPU map). The
                    // shared lock blocks until any in-flight map mutator
                    // (async mapIncremental / lasermapFovSegment) is done.
                    std::shared_lock<std::shared_mutex> map_read_lock(mtx_map_);
                    // Separate lock for SplineState mutations — serializes the
                    // IEKF's propRCP / updateIEKF writes against any other
                    // spline reader. Taken INSIDE the map read lock
                    // so the lock ordering (mtx_map_ before spline_mutex_) is
                    // consistent with the async path (no spline_mutex_ under
                    // mtx_map_ unique) — no cross-lock cycle possible.
                    std::lock_guard<std::mutex> spline_lock(spline_mutex_);
#ifdef RESPLE_USE_CUDA
                    // Use GPU path only once the map has actually been seeded
                    // (mapIncremental initializes the kd-tree first; until
                    // CudaMap::update() runs, fall back to the kd-tree path).
                    const bool use_gpu = !g_cuda_map.empty();
#else
                    [[maybe_unused]] constexpr bool use_gpu = false;
#endif
                    if (if_lidar_only) {
                        estimator_lo.propRCP(max_time_ns);
#ifdef RESPLE_USE_CUDA
                        if (use_gpu) {
                            estimator_lo.updateIEKFLiDAR(pt_meas, pt_neighbors_, &g_cuda_map, param.nn_thresh, param.coeff_cov, num_threads_, num_match_points_);
                        } else
#endif
                        {
                            estimator_lo.updateIEKFLiDAR(pt_meas, pt_neighbors_, &ikdtree, param.nn_thresh, param.coeff_cov, num_threads_, num_match_points_);
                        }
                        total_iekf_iterations_.fetch_add(estimator_lo.n_iter, std::memory_order_relaxed);
                    } else {
                        if (!imu_meas.empty()) {
                            max_time_ns = std::max(imu_meas.back().time_ns, max_time_ns);
                        }
                        while (!imu_meas.empty() && imu_meas.front().time_ns < spline->maxTimeNs() - spline->getKnotTimeIntervalNs()) {
                            imu_meas.pop_front();
                        }
                        estimator_lio.propRCP(max_time_ns);
#ifdef RESPLE_USE_CUDA
                        if (use_gpu) {
                            estimator_lio.updateIEKFLiDARInertial(pt_meas, pt_neighbors_, &g_cuda_map, param.nn_thresh, imu_meas, gravity, param.cov_acc, param.cov_gyro, param.coeff_cov, num_threads_, num_match_points_);
                        } else
#endif
                        {
                            estimator_lio.updateIEKFLiDARInertial(pt_meas, pt_neighbors_, &ikdtree, param.nn_thresh, imu_meas, gravity, param.cov_acc, param.cov_gyro, param.coeff_cov, num_threads_, num_match_points_);
                        }
                        total_iekf_iterations_.fetch_add(estimator_lio.n_iter, std::memory_order_relaxed);
                    }
                }
                {
                    // pointBodyToWorld reads spline; keep serialized with
                    // IEKF writes via spline_mutex_.
                    std::lock_guard<std::mutex> spline_lock(spline_mutex_);
                    #pragma omp parallel for num_threads(num_threads_)
                    for (size_t i = 0; i < pt_meas.size(); i++) {
                        PointData& pt_data = pt_meas[i];
                        Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w, pt_data.t_bl, pt_data.q_bl);
                    }
                    // Cache knot count for updateDiagnostics (may run on the
                    // executor thread; atomic read there is lock-free).
                    cached_spline_knots_.store(spline->numKnots(), std::memory_order_relaxed);
                }
                for (size_t i = 0; i < pt_meas.size(); i++) {
                    PointData& pt_data = pt_meas[i];
                    pc_world.points.push_back(pt_data.pt_w);
                    accum_nearest_points.push_back(std::move(pt_neighbors_[i]));
                }
                pt_meas.clear();
                if (spline->numKnots() > max_spl_knots) {
                    estimate_msgs::msg::Spline spline_msg;
                    {
                        std::lock_guard<std::mutex> spline_lock(spline_mutex_);
                        spline->getSplineMsg(spline_msg, std::max(int(max_spl_knots-1),0));
                        max_spl_knots = spline->numKnots();
                    }
                    estimate_msgs::msg::Estimate est_msg;
                    est_msg.spline = spline_msg;
                    est_msg.if_full_window.data = (max_spl_knots >= 4);
                    est_msg.runtime.data = 0;
                    pub_est->publish(est_msg);
                    if (!est_window_first_logged_.exchange(true)) {
                        RCLCPP_INFO(this->get_logger(),
                            "[RESPLE] first /est_window published (numKnots=%ld)",
                            spline->numKnots());
                    }
                }
                // Pose/odom/TF publish is INLINE and unconditional. Keeping
                // it out of the async-map-update lambda means the odom frame
                // keeps flowing even if the background map mutation is
                // wedged inside ikd-Tree (rebuild can stall on big trees,
                // and previously this also stalled processData itself
                // because the loop blocked on map_update_future_.wait()).
                publishPoseAndTf();

                if (max_time_ns >= t_last_map_upd + 100000000LL) {
                    // Wait for any prior async map update before swapping buffers.
                    // Gated on map_update_pending_ so the worker never reads the
                    // future until a prior launch was observed here.
                    //
                    // Use wait_for with a generous timeout instead of plain wait().
                    // If the lambda hangs (slow build, blocking I/O in PCL, kd-tree
                    // pathological case), plain wait() permanently locks the worker:
                    // no further IEKF cycles, no /localization/resple/odometry
                    // publishes, node looks alive but is silent. Skipping this map
                    // update cycle is the safer failure mode — the next cycle
                    // re-checks pending and will either wait again (lambda made
                    // progress) or skip again (still hung), keeping the worker
                    // responsive to shutdown signals at minimum. 5 seconds is well
                    // above the worst-case observed lambda duration (~150 ms at -O3
                    // on a well-loaded sim) but still bounded.
                    if (map_update_pending_.load(std::memory_order_acquire)) {
                        // Poll the future in 100ms slices instead of one 5s wait so
                        // the worker checks rclcpp::ok() and processing_active_
                        // between slices. Without this, a SIGINT delivered while
                        // the worker is mid-wait blocks shutdown for up to 5s
                        // (then SIGTERM fires, then SIGKILL — observed 15s+
                        // teardown latency in the master launch). Same overall
                        // 5s budget, just chunked.
                        constexpr auto kSlice = std::chrono::milliseconds(100);
                        constexpr int kSlices = 50;  // 50 × 100 ms = 5 s
                        bool completed = false;
                        for (int i = 0; i < kSlices; ++i) {
                            if (!processing_active_.load() || !rclcpp::ok()) {
                                // Shutdown requested mid-wait. Bail out of the
                                // worker loop entirely; the future destructor in
                                // on_deactivate / on_shutdown still waits for the
                                // lambda before reset, so we don't leak.
                                return;
                            }
                            if (map_update_future_.wait_for(kSlice)
                                != std::future_status::timeout) {
                                completed = true;
                                break;
                            }
                        }
                        if (!completed) {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "[RESPLE] async map update did not complete within 5s; "
                                "skipping this cycle's map update. Worker remains responsive. "
                                "If this fires repeatedly, the lambda is hung "
                                "(check for kd-tree infinite loop or PCL blocking I/O).");
                            // Bail out of the map-update branch entirely: do NOT swap
                            // buffers (the prior lambda still owns pc_world_bg_) and
                            // do NOT launch a new lambda (we'd leak the future-shared-
                            // state when the next assignment overwrites the still-
                            // running one). Continue accumulating in pc_world for
                            // next cycle.
                            continue;
                        }
                    }
                    pc_world_bg_.points.swap(pc_world.points);
                    accum_nearest_points_bg_.swap(accum_nearest_points);
                    pc_world.clear();
                    accum_nearest_points.clear();
                    // Cloud publish runs INLINE before the async kicks off so
                    // /current_scan keeps flowing even if the background map
                    // mutation wedges (rebuild, blocking I/O, etc.).
                    // publishPoseAndTf already ran inline above; the async
                    // lambda is now strictly map-mutation, no ROS I/O.
                    publishCurrentScan(pc_world_bg_);
                    // Set map_update_pending_ BEFORE launching so on_deactivate's
                    // read (after worker join) sees the pending state. The
                    // lambda clears it on exit so the next loop iteration's
                    // wait() can short-circuit if the prior async already
                    // finished.
                    map_update_pending_.store(true, std::memory_order_release);
                    map_update_future_ = std::async(std::launch::async, [this]() {
                        // Wrap the entire body. Any throw here (PCL OOM,
                        // kd-tree internal exception, GPU error) would
                        // otherwise:
                        //   (a) leave map_update_pending_ stuck at true →
                        //       every subsequent processData cycle blocks on
                        //       the (already-completed) future.wait();
                        //   (b) be silently captured by the future and dropped
                        //       by the destructor when the next cycle assigns
                        //       a new future to map_update_future_.
                        // Net effect: silent stop of map updates → kd-tree
                        // goes stale → IEKF k-NN starts returning bad neighbors
                        // → NaN cov / divergence / SIGSEGV downstream.
                        // Catch, log, and clear the pending flag so the next
                        // cycle proceeds cleanly.
                        try {
                            // mapIncremental / lasermapFovSegment / GPU sync
                            // all run under one unique_lock(mtx_map_) so the
                            // IEKF (shared_lock) never races against either
                            // the kd-tree mutators or the CUDA k-NN buffers.
                            // (g_cuda_map.update writes the same device-side
                            // d_buckets / d_sorted_points arrays that
                            // batch_search reads — running them concurrently
                            // produced an "illegal memory access" SIGABRT.)
                            std::unique_lock<std::shared_mutex> map_lock(mtx_map_);
                            mapIncremental(pc_world_bg_, accum_nearest_points_bg_);
                            lasermapFovSegment();
#ifdef RESPLE_USE_CUDA
                            ikdtree.PCL_Storage.clear();
                            ikdtree.flatten_safe(ikdtree.PCL_Storage, NOT_RECORD);
                            g_cuda_map.update(ikdtree.PCL_Storage.data(), ikdtree.PCL_Storage.size());
#endif
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "[RESPLE] async map update threw: %s. Map this cycle dropped; "
                                "next cycle will proceed.", e.what());
                        } catch (...) {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "[RESPLE] async map update threw unknown exception type. "
                                "Map this cycle dropped; next cycle will proceed.");
                        }
                        map_update_pending_.store(false, std::memory_order_release);
                    });
                    t_last_map_upd = max_time_ns;
                }
                
                // Update diagnostic metrics
                auto frame_end = std::chrono::high_resolution_clock::now();
                auto frame_duration = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start);
                total_computation_time_us_.fetch_add(
                    static_cast<uint64_t>(frame_duration.count()), std::memory_order_relaxed);
                frame_count_.fetch_add(1, std::memory_order_relaxed);

                // Update diagnostics at 1 Hz
                if ((this->now().nanoseconds() - last_process_ns_.load(std::memory_order_relaxed)) >= 1000000000LL) {
                    diagnostics_.force_update();
                    last_process_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
                }
            }
                if (!collected_any) {
                    ++hb_collect_false_count;
                    // Yield instead of busy-spinning. Without this the outer
                    // while did ~10M iterations/sec when collectMeasurements
                    // had nothing to return — pure CPU burn that competed
                    // with the IEKF for cycles. 1ms sleep keeps end-to-end
                    // latency well under the 10ms knot interval while
                    // freeing the core for sensor callbacks.
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RESPLE] processData iteration threw: %s. Worker continues. "
                    "If this fires repeatedly, the underlying bug needs fixing — "
                    "this catch only prevents silent worker-thread death.", e.what());
                // Brief sleep so we don't tight-loop if the throw fires every cycle.
                if (rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } catch (...) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RESPLE] processData iteration threw unknown exception type. "
                    "Worker continues.");
                if (rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
    // These counters are written by the worker thread (processData) and
    // read/reset by updateDiagnostics, which can run on the ROS executor thread
    // via the Updater's internal 1 Hz timer (not only from the worker's
    // force_update). Plain members were a data race; atomics make every access
    // well-defined. Computation time is accumulated in integer microseconds
    // because C++17 has no atomic<double> fetch_add. last_process_ns_ replaces
    // an rclcpp::Time member for the same reason.
    std::atomic<int64_t> last_process_ns_{0};
    std::atomic<size_t> frame_count_{0};
    std::atomic<uint64_t> total_computation_time_us_{0};
    std::atomic<size_t> total_iekf_iterations_{0};
    // Phase-0 hardening instrumentation: cached under spline_mutex_ by the
    // worker at the end of each IEKF cycle, read lock-free by updateDiagnostics
    // (which may run on the ROS executor thread via the Updater's internal
    // timer, not just from the worker's force_update). Atomic so the read is
    // not a data race; stale-by-one-cycle is acceptable.
    std::atomic<int64_t> cached_spline_knots_{0};
    // Diagnostic buffer depths. Refreshed by the worker each loop iteration and
    // read by updateDiagnostics, which can run on the executor thread via the
    // Updater's internal 1 Hz timer. Atomic so the cross-thread read is not a
    // data race on the worker-owned (imu_buff, pt_meas) / callback-written
    // (pc_buff) containers — the same race class REVIEW_FIXES #3 fixed for the
    // perf counters, here extended to the buffer-depth metrics. Stale-by-one-
    // cycle is acceptable for diagnostics.
    std::atomic<int64_t> cached_lidar_buf_{0};
    std::atomic<int64_t> cached_imu_buf_{0};
    std::atomic<int64_t> cached_pt_meas_{0};
    // Per-window baseline for IEKF numerical-failure deltas (published as a
    // per-second rate, not a cumulative count).
    uint64_t last_numerical_failures_lo_ = 0;
    uint64_t last_numerical_failures_lio_ = 0;
    
    // Lifecycle management
    std::atomic<bool> processing_active_;
    std::thread processing_thread_;
    
    // SaveMap action server
    using SaveMapAction = estimate_msgs::action::SaveMap;
    using GoalHandleSaveMap = rclcpp_action::ServerGoalHandle<SaveMapAction>;
    rclcpp_action::Server<SaveMapAction>::SharedPtr save_map_action_server_;
    std::thread save_map_thread_;
    std::mutex save_map_mutex_;

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
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr pub_start_time;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_imu_transform_ = false;
    bool have_lidar_transform_ = false;
    Eigen::Affine3d lidar_to_baselink_;
    geometry_msgs::msg::TransformStamped imu_to_baselink_;
    
    bool publish_tf, invert_tf;
    std::string frame_id;
    std::string odom_id;

    std::map<std::string, LidarConfig> lidars;
    float ds_lm_voxel;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_body;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last_ds;
    pcl::PointCloud<pcl::PointXYZINormal> pc_world;
    int point_filter_num = 1;
    int64_t time_offset = 0;

    std::vector<BoxPointType> cub_needrm;
    BoxPointType LocalMap_Points;
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> accum_nearest_points;
    // Per-frame parallel buffer for k-NN results (used to live in PointData::nearest_points).
    // Sized to pt_meas.size() before each IEKF call; results moved into accum_nearest_points after.
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> pt_neighbors_;
    double cube_len = 2000; 
    const float MOV_THRESHOLD = 1.5f;
    float det_range = 100.0;
    // Atomic so cross-thread reads (worker, async map-update lambda, IMU
    // callback, lidar callbacks) are not data races. All write sites already
    // happen under one of the existing mutexes; making these atomic only
    // removes the bare-bool unsynchronized reads. Implicit conversion +
    // assignment lets the rest of the code stay unchanged.
    std::atomic<bool> if_init_map{false};
    std::atomic<bool> localmap_initialized_{false};
    struct LidarData {
        Eigen::aligned_deque<Eigen::aligned_vector<pcl::PointXYZINormal>> pc_buff;
        std::deque<int64_t> t_buff;
        std::mutex mtx_pc;
        Eigen::aligned_deque<PointData> pt_buff;
        std::atomic<int64_t> last_t_ns{0};
    };
    std::map<std::string, LidarData> lidars_data;    
    Eigen::aligned_deque<PointData> pt_meas;    

    // Atomic: worker reads this in processData without m_buff. Set once during
    // readParameters() (configure phase) and not again, but the read race is
    // still a race; atomic resolves it cheaply.
    std::atomic<bool> if_lidar_only{false};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    Eigen::aligned_deque<ImuData> imu_buff;
    Eigen::aligned_deque<ImuData> imu_meas;
    Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_int_buff;    
    std::mutex m_buff;
    // Reader-writer lock guarding ikdtree contents:
    //   - shared (read) lock around findCorresp / Nearest_Search calls in the IEKF
    //   - unique (write) lock around Add_Points / Delete_Point_Boxes / Build / flatten
    // This lets multiple parallel Nearest_Search threads run concurrently but
    // serializes them against the async map update and SaveMap action.
    std::shared_mutex mtx_map_;
    // Serializes SplineState mutations (IEKF propRCP / updateIEKF /
    // collectMeasurements propRCP / pointBodyToWorld read). The async
    // map-update lambda no longer touches the spline (publishPoseAndTf
    // and publishCurrentScan run inline on the worker thread instead),
    // so today this mutex only serializes the worker's own multi-step
    // spline mutations. Kept distinct from mtx_map_ so that future
    // executor-thread spline readers can grab spline_mutex_ without
    // contending with kd-tree writers.
    //
    // Lock ordering: when both are held, mtx_map_ is always acquired
    // first, then spline_mutex_, to avoid deadlocks.
    std::mutex spline_mutex_;
    std::future<void> map_update_future_;
    // Gatekeeper for map_update_future_ access. The worker thread is the only
    // writer of the future object itself; on_deactivate/on_cleanup read it
    // only AFTER joining the worker, so the future access is safe in practice.
    // This atomic flag makes the intent explicit and also surfaces the
    // pending state to diagnostics. Set true immediately before std::async,
    // set false by the async lambda as its final step.
    std::atomic<bool> map_update_pending_{false};
    pcl::PointCloud<pcl::PointXYZINormal> pc_world_bg_;
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> accum_nearest_points_bg_;
    bool acc_ratio;
    Eigen::Vector3d cov_ba;
    Eigen::Vector3d cov_bg;
    Eigen::Vector<double, 6> cov_pose;        
    Eigen::Vector3d gravity;

    // Atomic: written under m_buff (initialization), read without m_buff
    // (worker's processData drain decision and initialization()'s early-return).
    // The write site at initialization() keeps m_buff because the IMU callback
    // depends on a coherent flip with imu_int_buff state — see comment there.
    std::atomic<bool> if_init_filter{false};
    // One-shot diagnostic flags (cuda-perf-nano post-1.5 additions). Each
    // exchange(true) prints once at first occurrence so the launch log
    // shows the IMU/LiDAR/IEKF startup sequence without spamming.
    std::atomic<bool> imu_first_logged_{false};
    std::atomic<bool> imu_count_logged_{false};
    std::atomic<bool> lidar_first_logged_{false};
    std::atomic<bool> init_complete_logged_{false};
    std::atomic<bool> iekf_first_logged_{false};
    std::atomic<bool> est_window_first_logged_{false};

    // One-shot diagnostic: prints the first time *any* LiDAR callback fires,
    // so the user can tell whether LiDAR data is actually flowing or whether
    // the topic name is wrong. Pairs with the IMU first-received log.
    void noteFirstLidar(const char* kind, const std::string& msg_frame_id, std::size_t pts) {
        if (!lidar_first_logged_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] First LiDAR scan received (kind=%s, frame_id='%s', points=%zu)",
                kind, msg_frame_id.c_str(), pts);
        }
    }
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
        std::lock_guard<std::mutex> lock(save_map_mutex_);
        if (save_map_thread_.joinable()) {
            save_map_thread_.join();
        }
        save_map_thread_ = std::thread{std::bind(&RESPLE::executeSaveMap, this, std::placeholders::_1), goal_handle};
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
            {
                std::unique_lock<std::shared_mutex> map_lock(mtx_map_);
                ikdtree.flatten_safe(ikdtree.PCL_Storage, NOT_RECORD);
                map_cloud->points = ikdtree.PCL_Storage;
            }
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
        publish_tf = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "odom/publish_tf", true);
        invert_tf = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "odom/invert_tf", false);
        frame_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "frame_id", "base_link");
        odom_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "odom/frame_id", "odom");
        
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

        pc_last_ds.reset(new pcl::PointCloud<pcl::PointXYZINormal>());


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
                // Pre-check that both frames are known to the buffer before
                // calling canTransform. canTransform itself logs a noisy
                // `Invalid frame ID "X" passed to canTransform - frame does
                // not exist` warning the moment either frame is missing
                // (typical at startup before the static TF has propagated).
                if (!tf_buffer_->_frameExists(this->frame_id) ||
                    !tf_buffer_->_frameExists(source_frame_id)) {
                    return false;
                }
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
                // Same pre-check as updateImuTransform — avoids tf2's
                // "frame does not exist" warning while the static TF
                // publisher is still coming up.
                if (!tf_buffer_->_frameExists(this->frame_id) ||
                    !tf_buffer_->_frameExists(source_frame_id)) {
                    return false;
                }
                geometry_msgs::msg::TransformStamped transform;
                if (tf_buffer_->canTransform(this->frame_id, source_frame_id,
                                                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
                        transform = tf_buffer_->lookupTransform(this->frame_id, source_frame_id, 
                                                                        rclcpp::Time(0));
                        lidar_to_baselink_ = tf2::transformToEigen(transform);
                        
                        have_lidar_transform_ = true;
                        // Populate sensor_origin_body for all lidars from the TF-based
                        // translation — used for true sensor-frame range in outlier gating.
                        for (auto& [name, lcfg] : lidars) {
                            lcfg.sensor_origin_body = lidar_to_baselink_.translation();
                        }
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

        // Transform linear acceleration (accounting for centripetal acceleration).
        // r = translation from base_link origin to IMU sensor; correction is ω×(ω×r).
        Eigen::Vector3d lin_accel(imu_raw->linear_acceleration.x,
                                  imu_raw->linear_acceleration.y,
                                  imu_raw->linear_acceleration.z);
        Eigen::Vector3d lin_accel_transformed = transform_eigen.rotation() * lin_accel
                                               + ang_vel_transformed.cross(ang_vel_transformed.cross(transform_eigen.translation()));

        imu->linear_acceleration.x = lin_accel_transformed[0];
        imu->linear_acceleration.y = lin_accel_transformed[1];
        imu->linear_acceleration.z = lin_accel_transformed[2];

        return imu;
    }
    
    void getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
      try {
        // Snapshot both flags ONCE under m_buff so they can't flip mid-callback.
        // Previously we double-checked if_init_filter — first unlocked, then
        // under the lock — and in LO mode the worker could flip if_init_filter
        // true between the two reads, sending this callback into the LIO
        // transformImu + push_back path that LO was never meant to run.
        //
        // Under m_buff (which the worker also holds while clearing the buffers
        // immediately before setting if_init_filter), the snapshot is coherent:
        // either the worker hasn't flipped the flag yet (we treat as pre-init
        // and buffer raw IMU), or it already flipped and released m_buff (we
        // early-return in LO mode, or run the LIO transform path).
        std::lock_guard<std::mutex> lock(m_buff);

        const bool init_done = if_init_filter;

        // LO mode: once initialized, drop IMU entirely. The subscription itself
        // is torn down only in on_deactivate / on_cleanup (tearing it down from
        // the worker raced the executor and crashed — bdab3dc fix).
        if (if_lidar_only && init_done) {
            return;
        }

        // One-shot confirmation that IMU data is actually flowing — pairs
        // with the "Waiting for N IMU samples" warn so users can tell at a
        // glance whether the bag is producing IMU or the topic is wrong.
        if (!imu_first_logged_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] First IMU sample received (frame_id='%s')",
                imu_msg->header.frame_id.c_str());
        }

        if (imu_int_buff.size() >= 2000) {
            imu_int_buff.erase(imu_int_buff.begin());
        }

        // Pre-init: accept raw IMU for gravity alignment. Gravity direction is
        // frame-independent for roll/pitch — the accelerometer measures g
        // regardless of the sensor's mounting frame.
        if (!init_done) {
            imu_int_buff.push_back(imu_msg);
            return;
        }

        // LIO mode, post-init: apply the base_link extrinsic.
        if (updateImuTransform(imu_msg->header.frame_id)) {
            sensor_msgs::msg::Imu::SharedPtr transformed_imu = transformImu(imu_msg, imu_to_baselink_);
            imu_int_buff.push_back(transformed_imu);
        } else {
            // Transform not yet available — pass through (assumes IMU already in base_link frame)
            imu_int_buff.push_back(imu_msg);
        }
      } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "[RESPLE] getImuCallback exception: %s", e.what());
      }
    }

    // Diagnostic updater callback
    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
    {
        // Snapshot the worker-written atomics once so the metrics below are
        // computed from a consistent set of values.
        const size_t frame_count = frame_count_.load(std::memory_order_relaxed);
        const uint64_t comp_time_us = total_computation_time_us_.load(std::memory_order_relaxed);
        const size_t iekf_iters = total_iekf_iterations_.load(std::memory_order_relaxed);

        // Calculate processing rate
        double time_elapsed = (this->now().nanoseconds() - last_process_ns_.load(std::memory_order_relaxed)) / 1e9;
        double processing_rate = (time_elapsed > 0) ? frame_count / time_elapsed : 0.0;
        double avg_computation_time = (frame_count > 0) ? (comp_time_us / 1000.0) / frame_count : 0.0;
        double avg_iekf_iters = (frame_count > 0) ? static_cast<double>(iekf_iters) / frame_count : 0.0;

        // Determine rate-based health level
        const double expected_rate = 20.0;  // Target: 20 Hz
        const double warn_threshold = 0.7 * expected_rate;  // 14 Hz
        const double error_threshold = 0.5 * expected_rate;  // 10 Hz

        uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        std::string msg = "System healthy";
        if (frame_count == 0) {
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            msg = "No frames processed yet";
        } else if (processing_rate < error_threshold) {
            level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            msg = "Processing rate critically low";
        } else if (processing_rate < warn_threshold) {
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            msg = "Processing rate below target";
        }

        // Add detailed metrics
        stat.add("Processing Rate (Hz)", processing_rate);
        stat.add("Target Rate (Hz)", expected_rate);
        stat.add("Frames Processed", static_cast<int>(frame_count));
        stat.add("Avg Computation Time (ms)", avg_computation_time);
        stat.add("Avg IEKF Iterations", avg_iekf_iters);
        stat.add("Num Threads", num_threads_);
        stat.add("Num Match Points", num_match_points_);

        // Buffer sizes: read the worker-maintained atomic caches instead of the
        // live containers (mutated by the worker / lidar callbacks) to avoid a
        // data race when this callback runs on the executor thread.
        stat.add("LiDAR Buffer Size", static_cast<int>(cached_lidar_buf_.load(std::memory_order_relaxed)));
        stat.add("IMU Buffer Size", static_cast<int>(cached_imu_buf_.load(std::memory_order_relaxed)));
        stat.add("Point Meas Buffer Size", static_cast<int>(cached_pt_meas_.load(std::memory_order_relaxed)));

        // Phase-0 instrumentation: spline growth + IMU staging buffer + IEKF
        // numerical-failure rate. These are the primary signals for deciding
        // whether the unbounded-knot / unbounded-buffer / silent-failure
        // hazards listed in CLAUDE.md actually fire in practice.
        stat.add("Spline Knots", static_cast<int>(cached_spline_knots_.load(std::memory_order_relaxed)));
        size_t imu_int_size = 0;
        {
            std::lock_guard<std::mutex> lock(m_buff);
            imu_int_size = imu_int_buff.size();
        }
        stat.add("IMU Staging Buffer Size", static_cast<int>(imu_int_size));

        // IEKF numerical-failure delta per window. Only one of the two
        // estimators is actually active per run (gated by if_lidar_only) but
        // both exist and both expose a counter — publish both so the non-active
        // one stays pinned at 0 and the active one accumulates.
        uint64_t fails_lo  = estimator_lo.num_numerical_failures_.load(std::memory_order_relaxed);
        uint64_t fails_lio = estimator_lio.num_numerical_failures_.load(std::memory_order_relaxed);
        uint64_t dfails_lo  = fails_lo  - last_numerical_failures_lo_;
        uint64_t dfails_lio = fails_lio - last_numerical_failures_lio_;
        last_numerical_failures_lo_  = fails_lo;
        last_numerical_failures_lio_ = fails_lio;
        stat.add("IEKF Numerical Failures (LO, cumulative)",  static_cast<int>(fails_lo));
        stat.add("IEKF Numerical Failures (LIO, cumulative)", static_cast<int>(fails_lio));
        stat.add("IEKF Numerical Failures (LO, last window)",  static_cast<int>(dfails_lo));
        stat.add("IEKF Numerical Failures (LIO, last window)", static_cast<int>(dfails_lio));

        // Out-of-spline-range query counter from Association::pointBodyToWorld.
        // Signals that the deskew loop saw a scan timestamp outside the current
        // knot window — potential for extrapolation / map poisoning.
        stat.add("Spline Out-of-Range Queries (cumulative)",
                 static_cast<int>(Association::out_of_range_queries_.load(std::memory_order_relaxed)));

        // Escalate to at least WARN if the IEKF rejected an update this window —
        // they are silent otherwise (only std::cerr, no ROS publish). Don't
        // overwrite a pre-existing ERROR.
        if ((dfails_lo > 0 || dfails_lio > 0)
            && level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            msg = "IEKF numerical update failures observed this window";
        }

        stat.summary(level, msg);

        // Reset counters for next period. Subtract exactly what was reported
        // (rather than store(0)) so increments the worker makes between the
        // snapshot above and here are not dropped — they carry into the next
        // window instead of being lost.
        frame_count_.fetch_sub(frame_count, std::memory_order_relaxed);
        total_computation_time_us_.fetch_sub(comp_time_us, std::memory_order_relaxed);
        total_iekf_iterations_.fetch_sub(iekf_iters, std::memory_order_relaxed);
    }

    template<typename T>
    void ousterLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ouster_msg_in)
    {
        // Wrap entire body: lidars.at()/lidars_data.at() throw std::out_of_range
        // for an unexpected frame, which propagates through the executor to
        // std::terminate → SIGABRT. PCL/TF calls can also throw. Convert to a
        // throttled warning so a single bad message can't take the node down.
        try {
        noteFirstLidar("Ouster", ouster_msg_in->header.frame_id,
                       ouster_msg_in->width * ouster_msg_in->height);
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
        int64_t stamp_ns = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds();
        if (stamp_ns < time_offset) return;  // skip early sim-time messages
        int64_t time_begin = stamp_ns - time_offset;
        LidarData& lidar_buffs = lidars_data.at(name);
        int64_t last_t_ns = lidar_buffs.last_t_ns.load();
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

        {
            std::lock_guard<std::mutex> lock(lidar_buffs.mtx_pc);
            lidar_buffs.pc_buff.push_back(pc_last->points);
            lidar_buffs.t_buff.push_back(time_begin);
        }
        lidar_buffs.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] ousterLidarCallback exception: %s", e.what());
        }
    }

    void livoxLidarCallback(const livox_ros_driver::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        try {
        noteFirstLidar("Mid70Avia", livox_msg_in->header.frame_id, livox_msg_in->point_num);
        std::string name = "Mid70Avia";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        // Belt-and-braces: gate on the actual vector size, not just the
        // point_num field. A misbehaving publisher (or transport corruption)
        // can ship point_num > 0 with an empty points vector → points[0]
        // below would be OOB. Doesn't affect Ouster (we don't take this
        // callback path), but is a real fix for the Livox variants.
        if (plsize == 0 || livox_msg_in->points.empty()) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        LidarData& lidar_buffs = lidars_data.at(name);
        int64_t last_t_ns = lidar_buffs.last_t_ns.load();
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

        {
            std::lock_guard<std::mutex> lock(lidar_buffs.mtx_pc);
            lidar_buffs.pc_buff.push_back(pc_transformed->points);
            lidar_buffs.t_buff.push_back(time_begin);
        }
        lidar_buffs.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] livoxLidarCallback exception: %s", e.what());
        }
    }

    void livoxLidar2Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        try {
        noteFirstLidar("HAP360", livox_msg_in->header.frame_id, livox_msg_in->point_num);
        std::string name = "HAP360";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        // Belt-and-braces: gate on the actual vector size, not just the
        // point_num field. A misbehaving publisher (or transport corruption)
        // can ship point_num > 0 with an empty points vector → points[0]
        // below would be OOB. Doesn't affect Ouster (we don't take this
        // callback path), but is a real fix for the Livox variants.
        if (plsize == 0 || livox_msg_in->points.empty()) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        LidarData& lidar_buffs = lidars_data.at(name);
        int64_t last_t_ns = lidar_buffs.last_t_ns.load();
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

        {
            std::lock_guard<std::mutex> lock(lidar_buffs.mtx_pc);
            lidar_buffs.pc_buff.push_back(pc_transformed->points);
            lidar_buffs.t_buff.push_back(time_begin);
        }
        lidar_buffs.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] livoxLidar2Callback exception: %s", e.what());
        }
    }

     void livoxAVIACallback(const livox_interfaces::msg::CustomMsg::SharedPtr livox_msg_in)
     {
        try {
        noteFirstLidar("AviaResple", livox_msg_in->header.frame_id, livox_msg_in->point_num);
        std::string name = "AviaResple";
        const LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        // Belt-and-braces: gate on the actual vector size, not just the
        // point_num field. A misbehaving publisher (or transport corruption)
        // can ship point_num > 0 with an empty points vector → points[0]
        // below would be OOB. Doesn't affect Ouster (we don't take this
        // callback path), but is a real fix for the Livox variants.
        if (plsize == 0 || livox_msg_in->points.empty()) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        LidarData& lidar_buffs = lidars_data.at(name);
        int64_t last_t_ns = lidar_buffs.last_t_ns.load();
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

        {
            std::lock_guard<std::mutex> lock(lidar_buffs.mtx_pc);
            lidar_buffs.pc_buff.push_back(pc_transformed->points);
            lidar_buffs.t_buff.push_back(time_begin);
        }
        lidar_buffs.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] livoxAVIACallback exception: %s", e.what());
        }
     }

    void hesaiLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr hesai_msg_in)
	{
        try {
        noteFirstLidar("Hesai", hesai_msg_in->header.frame_id,
                       hesai_msg_in->width * hesai_msg_in->height);
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
        LidarData& lidar_buffs_hesai = lidars_data.at(name);
        int64_t last_t_ns = lidar_buffs_hesai.last_t_ns.load();
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
                rclcpp::Time timestamp_ros(static_cast<uint32_t>(timestamp_s), static_cast<uint32_t>(timestamp_ns * 1.0e9),
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

        {
            std::lock_guard<std::mutex> lock(lidar_buffs_hesai.mtx_pc);
            lidar_buffs_hesai.pc_buff.push_back(pc_transformed->points);
            lidar_buffs_hesai.t_buff.push_back(time_begin);
        }
        lidar_buffs_hesai.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] hesaiLidarCallback exception: %s", e.what());
        }
	}

    void livoxMid360BoxiCallback(const sensor_msgs::msg::PointCloud2::SharedPtr livox_msg_in)
	{
        try {
        noteFirstLidar("Mid360Boxi", livox_msg_in->header.frame_id,
                       livox_msg_in->width * livox_msg_in->height);
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
        LidarData& lidar_buffs_boxi = lidars_data.at(name);
        int64_t last_t_ns = lidar_buffs_boxi.last_t_ns.load();
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

        {
            std::lock_guard<std::mutex> lock(lidar_buffs_boxi.mtx_pc);
            lidar_buffs_boxi.pc_buff.push_back(pc_transformed->points);
            lidar_buffs_boxi.t_buff.push_back(time_begin);
        }
        lidar_buffs_boxi.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] livoxMid360BoxiCallback exception: %s", e.what());
        }
	}

    // Pose / odom / TF only — no point cloud, no map access. Safe to call
    // every IEKF iteration on the worker thread; intentionally separated
    // from publishCurrentScan so the odom frame keeps flowing even when
    // the background map mutation lambda is wedged inside ikd-Tree.
    int64_t last_pose_pub_time_ns_ = std::numeric_limits<int64_t>::min();

    void publishPoseAndTf()
    {
        int64_t pose_time_ns = spline->maxTimeNs();
        // Skip duplicates: the inner collectMeasurements loop can fire many
        // times before the spline advances, and tf2 / nav listeners reject
        // identical-stamp transforms anyway.
        if (pose_time_ns <= last_pose_pub_time_ns_) return;
        last_pose_pub_time_ns_ = pose_time_ns;

        Eigen::Vector3d t_pose = spline->itpPosition(pose_time_ns);
        Eigen::Quaterniond q_pose;
        spline->itpQuaternion(pose_time_ns, &q_pose);

        geometry_msgs::msg::PoseStamped pose_msg =
            CommonUtils::pose2msg(odom_id, pose_time_ns, t_pose, q_pose);
        pub_pose->publish(pose_msg);

        // Pull the IEKF posterior covariance from the active estimator and
        // project it through the spline Jacobian so downstream consumers
        // (robot_localization, sierra) actually see per-step uncertainty
        // rather than the static cov_pose YAML diagonal.
        const Eigen::Matrix<double, 6, 6> P_pose =
            if_lidar_only ? estimator_lo.getLastPoseCovariance()
                          : estimator_lio.getLastPoseCovariance();
        const Eigen::Matrix<double, 6, 6> P_twist =
            if_lidar_only ? estimator_lo.getLastTwistCovariance()
                          : estimator_lio.getLastTwistCovariance();

        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
        pose_cov_msg.header.frame_id = odom_id;
        pose_cov_msg.header.stamp = rclcpp::Time(pose_time_ns);
        pose_cov_msg.pose.pose = pose_msg.pose;
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                pose_cov_msg.pose.covariance[r * 6 + c] = P_pose(r, c);
        pub_pose_cov->publish(pose_cov_msg);

        // Odometry (pose + twist) for EKF fusion — twist from spline derivative
        // gives the odom EKF a smooth velocity between pose updates.
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = odom_id;
        odom_msg.header.stamp = rclcpp::Time(pose_time_ns);
        odom_msg.child_frame_id = frame_id;
        odom_msg.pose.pose = pose_cov_msg.pose.pose;
        for (int i = 0; i < 36; ++i)
            odom_msg.pose.covariance[i] = pose_cov_msg.pose.covariance[i];

        Eigen::Vector3d v_world = spline->itpPosition<1>(pose_time_ns);
        Eigen::Vector3d v_body = q_pose.inverse() * v_world;
        odom_msg.twist.twist.linear.x = v_body.x();
        odom_msg.twist.twist.linear.y = v_body.y();
        odom_msg.twist.twist.linear.z = v_body.z();

        Eigen::Vector3d w_body;
        spline->itpQuaternion(pose_time_ns, nullptr, &w_body);
        odom_msg.twist.twist.angular.x = w_body.x();
        odom_msg.twist.twist.angular.y = w_body.y();
        odom_msg.twist.twist.angular.z = w_body.z();

        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                odom_msg.twist.covariance[r * 6 + c] = P_twist(r, c);

        pub_odom->publish(odom_msg);

        if (publish_tf) {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.transform.translation.x = pose_msg.pose.position.x;
            transformStamped.transform.translation.y = pose_msg.pose.position.y;
            transformStamped.transform.translation.z = pose_msg.pose.position.z;
            transformStamped.transform.rotation = pose_msg.pose.orientation;

            tf2::Transform transform_tf;
            tf2::fromMsg(transformStamped.transform, transform_tf);
            if (invert_tf) transform_tf = transform_tf.inverse();
            tf2::toMsg(transform_tf, transformStamped.transform);

            transformStamped.header.stamp = pose_msg.header.stamp;
            if (invert_tf) {
                transformStamped.header.frame_id = frame_id;
                transformStamped.child_frame_id  = odom_id;
            } else {
                transformStamped.header.frame_id = odom_id;
                transformStamped.child_frame_id  = frame_id;
            }
            br->sendTransform(transformStamped);
        }
    }

    void publishCurrentScan(pcl::PointCloud<pcl::PointXYZINormal>& pc)
    {
        int size = pc.points.size();
        laser_cloud_world_reusable_->clear();
        laser_cloud_world_reusable_->points.resize(size);
        for (int i = 0; i < size; i++) {
            laser_cloud_world_reusable_->points[i].x = pc.points[i].x;
            laser_cloud_world_reusable_->points[i].y = pc.points[i].y;
            laser_cloud_world_reusable_->points[i].z = pc.points[i].z;
            laser_cloud_world_reusable_->points[i].intensity = pc.points[i].curvature;
        }
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
        // Discard pre-init backlog: when the bag started before this node
        // was ready (the common case), pc_buff + pt_buff have accumulated
        // several seconds of stale scans by the time gravity alignment
        // finishes. Anchoring the spline at pt_buff.front() then locks
        // in a permanent real-time lag — the estimator processes at
        // exactly the bag rate and never catches up, so visualization
        // drifts further behind the actual robot pose every second.
        //
        // The latest LiDAR/IMU arrivals define "now" — both topics are
        // confirmed active by this point (we wouldn't be here without
        // first IMU + first LiDAR). Drop everything in pc_buff/t_buff/
        // pt_buff older than a short window before the most recent
        // point. 200ms = ~2 Avia scans, enough geometric overlap for
        // the first IEKF iteration without dragging in stale history.
        if (!if_init_filter) {
            constexpr int64_t init_lag_window_ns = 200'000'000;  // 200ms
            int64_t latest_t_ns = std::numeric_limits<int64_t>::min();
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                if (!lidar_data.pt_buff.empty()) {
                    latest_t_ns = std::max(latest_t_ns, lidar_data.pt_buff.back().time_ns);
                }
                // pc_buff/t_buff are pushed in the LiDAR callback; t_buff
                // entries are scan-start timestamps (header.stamp).
                std::lock_guard<std::mutex> lock(lidar_data.mtx_pc);
                if (!lidar_data.t_buff.empty()) {
                    latest_t_ns = std::max(latest_t_ns, lidar_data.t_buff.back());
                }
            }
            int64_t cutoff = latest_t_ns - init_lag_window_ns;
            size_t total_pt_dropped = 0;
            size_t total_pc_dropped = 0;
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.pt_buff.empty() &&
                       lidar_data.pt_buff.front().time_ns < cutoff) {
                    lidar_data.pt_buff.pop_front();
                    ++total_pt_dropped;
                }
                std::lock_guard<std::mutex> lock(lidar_data.mtx_pc);
                while (!lidar_data.t_buff.empty() &&
                       lidar_data.t_buff.front() < cutoff) {
                    lidar_data.pc_buff.pop_front();
                    lidar_data.t_buff.pop_front();
                    ++total_pc_dropped;
                }
            }
            if (total_pt_dropped > 0 || total_pc_dropped > 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] dropped %zu pt_buff + %zu pc_buff stale entries "
                    "(kept last %ld ms ending at %ld) to start near real time",
                    total_pt_dropped, total_pc_dropped,
                    init_lag_window_ns / 1'000'000, latest_t_ns);
            }
        }
        int64_t start_t_ns = std::numeric_limits<int64_t>::max();
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            start_t_ns = std::min(start_t_ns, std::max(lidar_data.pt_buff.front().time_ns, int64_t(0)));
        }
        if (!if_init_filter) {
            Eigen::Quaterniond q_WI = Eigen::Quaterniond::Identity();

            // Always use IMU for gravity alignment, even in LO mode.
            // This ensures the spline starts gravity-aligned (z = up).
            {
                int buff_size;
                int n_imu;
                Eigen::Vector3d gravity_mean;
                double accel_variance = 0.0;
                {
                    std::unique_lock<std::mutex> imu_lock(m_buff);
                    buff_size = imu_buff.size();

                    // Wait for sufficient IMU samples
                    if (buff_size < imu_init_num_samples_) {
                        imu_lock.unlock();
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Waiting for %d IMU samples for gravity alignment (current: %d)",
                            imu_init_num_samples_, buff_size);
                        return false;
                    }

                    // One-shot: positive confirmation that we hit the sample
                    // count, even if the variance check below later fails. Pairs
                    // with the "Waiting for N IMU samples" warn — without this
                    // log, a stuck variance check looks identical to "no IMU".
                    if (!imu_count_logged_.exchange(true)) {
                        RCLCPP_INFO(this->get_logger(),
                            "[RESPLE] Collected %d IMU samples for gravity alignment "
                            "(buffer size: %d)",
                            imu_init_num_samples_, buff_size);
                    }

                    // Scan ALL n_imu-wide windows in the buffer and pick
                    // the one with the lowest accelerometer variance.
                    // Previously we always took the tail; if the bag
                    // started with the robot moving (or — common with our
                    // delayed-launch workflow — the latest samples happen
                    // to land mid-motion), the tail would fail the
                    // variance check and the spline orientation ended up
                    // anchored to a noisy gravity estimate, tilting the
                    // map. Picking the quietest window in the buffer
                    // recovers a clean gravity even when init lands
                    // mid-bag.
                    n_imu = std::min(imu_init_num_samples_, buff_size);
                    const int n_windows = buff_size - n_imu + 1;
                    Eigen::Vector3d best_mean = Eigen::Vector3d::Zero();
                    double best_var = std::numeric_limits<double>::infinity();
                    int best_start = 0;
                    for (int w = 0; w < n_windows; ++w) {
                        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                        for (int i = 0; i < n_imu; ++i) {
                            sum += imu_buff.at(w + i).accel;
                        }
                        Eigen::Vector3d mean = sum / n_imu;
                        double var = 0.0;
                        for (int i = 0; i < n_imu; ++i) {
                            var += (imu_buff.at(w + i).accel - mean).squaredNorm();
                        }
                        var /= n_imu;
                        if (var < best_var) {
                            best_var = var;
                            best_mean = mean;
                            best_start = w;
                        }
                    }
                    gravity_mean = best_mean;
                    accel_variance = best_var;

                    if (accel_variance > imu_init_max_variance_) {
                        imu_lock.unlock();
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "IMU readings too noisy for initialization (best window variance: %.4f > %.4f, "
                            "scanned %d windows). Ensure robot is stationary at some point during launch "
                            "or increase imu_init_max_variance parameter.",
                            accel_variance, imu_init_max_variance_, n_windows);
                        return false;
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "[RESPLE] gravity window: picked samples [%d..%d] of %d "
                        "(variance %.4f, scanned %d windows)",
                        best_start, best_start + n_imu - 1, buff_size,
                        accel_variance, n_windows);

                    // Clean up old IMU data
                    while (!imu_buff.empty() && imu_buff.front().time_ns < start_t_ns) {
                        imu_buff.pop_front();
                    }
                }

                // Initialize orientation from gravity
                Eigen::Vector3d gravity_ave = gravity_mean.normalized() * 9.81;
                Eigen::Matrix3d R0 = CommonUtils::g2R(gravity_ave);
                double yaw = CommonUtils::R2ypr(R0).x();
                R0 = CommonUtils::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
                Eigen::Quaterniond q0(R0);
                q_WI = Quater::positify(q0);
                gravity = q_WI * gravity_ave;

                RCLCPP_INFO(this->get_logger(),
                    "Gravity alignment successful (samples: %d, variance: %.4f, mode: %s)",
                    n_imu, accel_variance, if_lidar_only ? "LO" : "LIO");
            }

            // Initialize the filter BEFORE flipping if_init_filter — this way
            // when the callback acquires m_buff and sees if_init_filter=true,
            // the SplineState is already fully constructed.
            initFilter(start_t_ns, Eigen::Vector3d(0, 0, 0), q_WI);

            // Flip if_init_filter under m_buff so getImuCallback's snapshot
            // observes a coherent pre→post transition. In LO mode we also
            // clear the IMU buffers here — callbacks that already acquired
            // m_buff and are waiting (or about to run) will observe the
            // post-init state and either early-return (LO) or take the
            // LIO transform path.
            //
            // We do NOT call sub_imu.reset() here: tearing down a live
            // Subscription from this worker thread races the executor's
            // dispatcher (MultiThreadedExecutor on the sensor callback group)
            // and can use-after-free → SIGSEGV. Instead, getImuCallback
            // early-returns once (if_lidar_only && if_init_filter) is true,
            // and the subscription is destroyed in on_deactivate / on_cleanup.
            {
                std::lock_guard<std::mutex> lock(m_buff);
                if (if_lidar_only) {
                    imu_int_buff.clear();
                    imu_buff.clear();
                    RCLCPP_INFO(this->get_logger(), "LO mode: IMU input disabled after gravity alignment");
                }
                if_init_filter = true;
            }

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
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + 100000000LL) {
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
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + 100000000LL) {
                        Association::pointBodyToWorld(start_t_ns, spline, lidar_data.pt_buff[i].pt,
                            pc_world.points[world_i], lidar_data.pt_buff[i].t_bl, lidar_data.pt_buff[i].q_bl);
                        world_i++;
                    } else {
                        break;
                    }
                }
            }
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns < start_t_ns + 100000000LL) {
                    lidar_data.pt_buff.pop_front();
                }
            }
            {
                std::unique_lock<std::shared_mutex> map_lock(mtx_map_);
                ikdtree.Build(pc_world.points);
#ifdef RESPLE_USE_CUDA
                // Initial GPU map sync from the freshly-built kd-tree.
                ikdtree.PCL_Storage.clear();
                ikdtree.flatten_safe(ikdtree.PCL_Storage, NOT_RECORD);
                g_cuda_map.update(ikdtree.PCL_Storage.data(), ikdtree.PCL_Storage.size());
#endif
            }
            pc_world.clear();
            if_init_map = true;
        }
        return false;
    }

    bool collectMeasurements()
    {
        int64_t pt_min_time = std::numeric_limits<int64_t>::max();
        int64_t pt_max_time = std::numeric_limits<int64_t>::max();
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            if (lidar_data.pt_buff.empty()) {
                return false;
            }
            pt_min_time = std::min(pt_min_time, lidar_data.pt_buff.front().time_ns);
            pt_max_time = std::min(pt_max_time, lidar_data.pt_buff.back().time_ns);
        }
        if (pt_max_time <= spline->maxTimeNs() + dt_ns) {
            return false;
        }
        if (!if_lidar_only && (imu_buff.empty() || imu_buff.back().time_ns <= spline->maxTimeNs())) {
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
        // Pure read: interpolate position+orientation at t_ns and project the
        // LiDAR mount offset.
        //
        // Previously this called propRCP(t_ns), which mutates Estimator::cov_rcp
        // (adds cov_sys) and may extend the spline. The only caller is
        // lasermapFovSegment, which always passes spline->maxTimeNs() — so the
        // spline never grew, but cov_rcp got bumped on every map-update tick,
        // and the mutation happened from the async map-update lambda WITHOUT
        // holding spline_mutex_ (only mtx_map_ unique). It happened to be
        // race-free because the IEKF needs mtx_map_ shared and so was blocked,
        // but the implicit contract was fragile. Removing the side effect
        // makes the lock discipline correct: this function is now const-like.
        Eigen::Quaterniond orient_interp;
        Eigen::Vector3d t_interp = spline->itpPosition(t_ns);
        spline->itpQuaternion(t_ns, &orient_interp);
        Eigen::Vector3d t = orient_interp * t_bl + t_interp;
        return t;
    }

    void lasermapFovSegment()
    {
        cub_needrm.shrink_to_fit();
        Eigen::Vector3d pos_lidar_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        // lowest(), not min(): numeric_limits<double>::min() is the smallest
        // POSITIVE normal (~2.2e-308), not the most negative double. With
        // min() the .max() reduction below silently fails for any pos with
        // negative components, biasing the local-map cube ~500m off-axis.
        Eigen::Vector3d pos_lidar_max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest());
        for (const auto& [lidar_name, lidar] : lidars) {
            Eigen::Vector3d pos_lidar = getPositionLiDAR(spline->maxTimeNs(), lidar.t_bl);
            pos_lidar_min = pos_lidar_min.array().min(pos_lidar.array()).matrix();
            pos_lidar_max = pos_lidar_max.array().max(pos_lidar.array()).matrix();
        }
        if (!localmap_initialized_){
            for (int i = 0; i < 3; i++){
                LocalMap_Points.vertex_min[i] = pos_lidar_min(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_lidar_max(i) + cube_len / 2.0;
            }
            localmap_initialized_ = true;
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

        // Caller (async map task) holds mtx_map_ unique_lock for the entire
        // sequence of mapIncremental + lasermapFovSegment + GPU sync.
        if(cub_needrm.size() > 0) {
            ikdtree.Delete_Point_Boxes(cub_needrm);
        }
    }

    void mapIncremental(pcl::PointCloud<pcl::PointXYZINormal>& pc,
                        std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& nearest_pts)
    {
        Eigen::aligned_vector<pcl::PointXYZINormal> PointToAdd;
        Eigen::aligned_vector<pcl::PointXYZINormal> PointNoNeedDownsample;
        int feats_down_size = pc.points.size();
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        // Defense: count non-finite (NaN/Inf) points encountered. ikd-Tree's
        // Add_by_point recursion uses calc_dist (Euclidean diff) and compares
        // with `<` against zero/threshold. If a coordinate is NaN, every
        // comparison is false, the recursion takes the wrong branch each
        // level, and downsample-mid-point math (floor(x/ds)*ds) yields NaN —
        // which then writes NaN into the tree's bbox and into Downsample
        // results. From there, Search_by_range's bbox checks all return
        // false, and tree state degrades silently. Skip non-finite points
        // upstream of the kd-tree; surface a counter so the operator sees it.
        size_t nan_skipped = 0;
        for(int i = 0; i < feats_down_size; i++) {
            const pcl::PointXYZINormal& point = pc.points[i];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                ++nan_skipped;
                continue;
            }
            if (!nearest_pts[i].empty()) {
                const Eigen::aligned_vector<pcl::PointXYZINormal> &points_near = nearest_pts[i];
                bool need_add = true;
                pcl::PointXYZINormal downsample_result, mid_point;

                mid_point.x = floor(point.x/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.y = floor(point.y/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.z = floor(point.z/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                if (fabs(points_near[0].x - mid_point.x) > 0.866 * ds_lm_voxel || fabs(points_near[0].y - mid_point.y) > 0.866 * ds_lm_voxel || fabs(points_near[0].z - mid_point.z) > 0.866 * ds_lm_voxel){
                    PointNoNeedDownsample.emplace_back(pc.points[i]);
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
        if (nan_skipped > 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] mapIncremental: skipped %zu non-finite point(s) this scan. "
                "Most often a sign of bad spline-extrapolation in pointBodyToWorld; "
                "check Association::out_of_range_queries_ counter and DLIO/Ouster timestamps.",
                nan_skipped);
        }
        // Caller (async map task) holds mtx_map_ unique_lock.
        ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
    }

};

// Crash handler: prints a backtrace to stderr on fatal signals, then restores
// the default handler and re-raises so the kernel can still produce a core
// dump / the launch system sees the original signal. Installed in main().
//
// Added 2026-04-21: RESPLE was dying with exit code -11 immediately after
// gravity alignment in sim, with no prior log context. Launch-level respawn
// kept the stack up but each restart crashed the same way deterministically.
// This handler produces the call stack needed to pinpoint the instruction.
//
// Not signal-safe in the strictest sense (fprintf allocates, backtrace_symbols
// can malloc), but it's the last thing the process does before re-raising, so
// any heap corruption here is a non-issue. backtrace_symbols_fd is the
// async-signal-safer variant we use for the actual frame output.
static void respleCrashHandler(int sig)
{
    constexpr int kMaxFrames = 64;
    void *addrs[kMaxFrames];
    int n = backtrace(addrs, kMaxFrames);
    const char *name = (sig == SIGSEGV) ? "SIGSEGV" :
                       (sig == SIGABRT) ? "SIGABRT" :
                       (sig == SIGBUS)  ? "SIGBUS"  :
                       (sig == SIGFPE)  ? "SIGFPE"  :
                       (sig == SIGILL)  ? "SIGILL"  : "unknown";
    // Write the banner via the STDERR fd directly so it appears even when the
    // stdio buffers are corrupted (common after a memory-safety violation).
    dprintf(STDERR_FILENO, "\n=== RESPLE crash handler: caught %s (%d), %d frames ===\n",
            name, sig, n);
    backtrace_symbols_fd(addrs, n, STDERR_FILENO);
    dprintf(STDERR_FILENO, "=== end RESPLE backtrace ===\n");
    // Restore default disposition and re-raise so the kernel emits a core
    // and the parent (ros2 launch) observes the original signal.
    signal(sig, SIG_DFL);
    raise(sig);
}

// Real entry point, compiled once into libresple. The thin RESPLE_main.cpp
// wrapper (the executable's only TU) just forwards to this, so the heavy node
// source is no longer compiled twice (lib + executable).
int respleMain(int argc, char *argv[])
{
    // Install before rclcpp::init so crashes during subscriber/publisher
    // construction also produce a trace.
    signal(SIGSEGV, respleCrashHandler);
    signal(SIGABRT, respleCrashHandler);
    signal(SIGBUS,  respleCrashHandler);
    signal(SIGFPE,  respleCrashHandler);
    signal(SIGILL,  respleCrashHandler);

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    rclcpp::init(argc, argv);
    
    // Phase 4: Lifecycle node initialization
    rclcpp::NodeOptions options;
    auto node = std::make_shared<RESPLE>(options);
    RCLCPP_INFO_STREAM(node->get_logger(), "RESPLE LifecycleNode created");

    // Transition to configured state. Wrap each lifecycle call so an exception
    // thrown inside on_configure/on_activate is logged here instead of
    // propagating to terminate (where the crash handler prints a backtrace
    // but the user has no clue WHY it threw).
    {
        auto state = node->configure();
        const auto label = state.label();
        RCLCPP_INFO_STREAM(node->get_logger(),
            "RESPLE configure() returned id=" << static_cast<int>(state.id())
            << " label=" << label);
        if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_FATAL_STREAM(node->get_logger(),
                "RESPLE configure() failed (state=" << label
                << "); refusing to activate. Exiting non-zero.");
            rclcpp::shutdown();
            return 2;
        }
    }

    // Transition to active state (starts processing)
    {
        auto state = node->activate();
        const auto label = state.label();
        RCLCPP_INFO_STREAM(node->get_logger(),
            "RESPLE activate() returned id=" << static_cast<int>(state.id())
            << " label=" << label);
        if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_FATAL_STREAM(node->get_logger(),
                "RESPLE activate() failed (state=" << label
                << "); the executor would have spun on a half-init node. Exiting non-zero.");
            rclcpp::shutdown();
            return 3;
        }
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "RESPLE active - entering executor spin");

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();

    // Cleanup on shutdown (only if context still valid)
    bool sigint_path = !rclcpp::ok();
    if (!sigint_path) {
        RCLCPP_INFO(node->get_logger(), "Gracefully shutting down RESPLE...");
        node->deactivate();
        node->cleanup();
        node->shutdown();
        exec.remove_node(node->get_node_base_interface());
        rclcpp::shutdown();
        return 0;
    }

    // SIGINT path: the executor is no longer spinning and lifecycle
    // transitions cannot be driven through the state machine. Run
    // on_shutdown directly to stop the processing thread, then bail
    // out hard — exec.remove_node / rclcpp::shutdown / static
    // destructors all have observed wedge modes (CUDA teardown,
    // ikd-Tree rebuild thread join, FastDDS atexit) that cause the
    // launcher to escalate SIGINT → SIGTERM → SIGKILL. on_shutdown
    // already released the ROS-visible state we care about.
    node->on_shutdown(node->get_current_state());
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(0);
}

RCLCPP_COMPONENTS_REGISTER_NODE(RESPLE)
