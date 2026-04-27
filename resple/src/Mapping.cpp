#include <thread>
#include <iostream>
#include <queue>
#include <string>
#include <cmath>
#include <atomic>
#include <cstdio>
#include <csignal>
#include <execinfo.h>
#include <unistd.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include "livox_ros_driver/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"
#include "estimate_msgs/msg/calib.hpp"
#include "estimate_msgs/msg/estimate.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.hpp>

#include "SplineState.h"

template<typename PointType>
class MappingBase
{
  public:

    std::mutex mtx;
    LidarConfig lidar;
    MappingBase(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config,
                rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr)
                : lidar(lidar_config)
                , lidar_qos(rclcpp::SensorDataQoS().keep_last(100).best_effort())
                , node_handle_(nh)
    {
        if (sensor_cb) sub_opt.callback_group = sensor_cb;

        // Read frame ID parameters
        frame_id = CommonUtils::readParam<std::string>(nh->get_node_parameters_interface(), "frame_id", "base_link");
        map_id = CommonUtils::readParam<std::string>(nh->get_node_parameters_interface(), "map/frame_id", "map");
        num_threads_ = CommonUtils::readParam<int>(nh->get_node_parameters_interface(), "num_threads", 5);

        RCLCPP_INFO(nh->get_logger(), "Frame IDs -  map: %s, body: %s", 
                    map_id.c_str(), frame_id.c_str());        

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        have_lidar_transform_ = false;
        have_imu_transform_ = false;
        lidar_to_baselink_ = Eigen::Affine3d::Identity();
        imu_to_baselink_ = Eigen::Affine3d::Identity();

        pub_global_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("global_map",
            rclcpp::QoS(2).reliable());
        ds_filter_each_scan.setLeafSize(0.2, 0.2, 0.2);
        pc_last.reset(new typename pcl::PointCloud<PointType>());
        pc_last_ds.reset(new typename pcl::PointCloud<PointType>());
        pc.reset(new typename pcl::PointCloud<PointType>());
    }

    Eigen::Affine3d getLidarToBaselink()
    {
        return lidar_to_baselink_;
    }

    Eigen::Affine3d getImuToBaselink()
    {
        return imu_to_baselink_;
    }

    void processScan(SplineState* spl, const int64_t spl_window_st_ns)
    {
        (void)spl_window_st_ns;
        rclcpp::Rate rate(20);
        while (true) {
            // Pull the next cloud out of the buffer entirely under the lock.
            // The previous code read pc_L_buff.front() before taking mtx,
            // which raced the callback's buffer-cap pop_front: front()
            // returns a reference that pop_front invalidates → UAF read of
            // the cloud header.
            pcl::PointCloud<PointType> front_cloud;
            {
                std::unique_lock<std::mutex> lock(mtx);
                if (pc_L_buff.empty()) break;
                const auto& front = pc_L_buff.front();
                // points.back() on an empty cloud is UB; guard and drop.
                if (front.points.empty()) {
                    pc_L_buff.pop_front();
                    continue;
                }
                const int64_t t_end_ns = front.header.stamp +
                    int64_t(front.points.back().intensity * float(1e6));
                if (t_end_ns < spl->minTimeNs()) {
                    pc_L_buff.pop_front();
                    continue;
                }
                if (t_end_ns > spl->maxTimeNs()) {
                    // Front spans beyond the current spline window — retry
                    // next tick once more knots are available.
                    lock.unlock();
                    rate.sleep();
                    break;
                }
                // Move the cloud out of the deque (deque move of
                // pcl::PointCloud is a pointer swap) so the transform +
                // publish run without blocking the callback.
                front_cloud = std::move(pc_L_buff.front());
                pc_L_buff.pop_front();
            }
            transformCloud(front_cloud, spl, pc);
            publishMap(pc, pub_global_map);
        }
    }

    void publishMap(const typename pcl::PointCloud<PointType>::Ptr& pcs,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const
    {
        // RCLCPP_INFO(node_handle_->get_logger(), "publishMap");
        sensor_msgs::msg::PointCloud2 msgs;
        pcl::toROSMsg(*pcs, msgs);
        msgs.header.frame_id = map_id;
        publisher->publish(msgs);
    }

    bool updateTransform(std::string source_frame_id)
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
                        Eigen::Translation3d translation(lidar.t_lb);
                        Eigen::Affine3d imu_to_lidar = translation * lidar.q_lb;
                        imu_to_baselink_ = lidar_to_baselink_ * imu_to_lidar;
                        have_imu_transform_ = true;                        
                        RCLCPP_INFO(node_handle_->get_logger(), "[Mapping] Got LiDAR transform: %s -> %s", 
                                source_frame_id.c_str(), this->frame_id.c_str());
                    } else {
                        RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), 5000, 
                                            "[Mapping] Waiting for LiDAR transform: %s -> %s", 
                                            source_frame_id.c_str(), this->frame_id.c_str());
                        return false;
                    }
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), 5000, 
                                    "[Mapping] LiDAR transform exception: %s", ex.what());
                return false;
            }
        }

        return true;
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map;

    PointType transformPoint(int64_t time_ns, const SplineState* spl, const PointType& pt_in) const
    {
        Eigen::Quaterniond q_itp;
        Eigen::Vector3d t_itp;
        spl->itpQuaternion(time_ns, &q_itp);
        t_itp = spl->itpPosition(time_ns);
        Eigen::Vector3d p_body(pt_in.x, pt_in.y, pt_in.z);
        Eigen::Vector3d p_imu(lidar.q_bl * p_body + lidar.t_bl);
        Eigen::Vector3d p_global(q_itp * p_imu + t_itp);
        PointType point_world;
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = pt_in.intensity;
        point_world.curvature = pt_in.curvature;
        return point_world;
    }

    void transformCloud(const typename pcl::PointCloud<PointType>& pc_in, SplineState* spl,
                       typename pcl::PointCloud<PointType>::Ptr pc_out) const
    {
        int64_t time_begin = rclcpp::Time(pc_in.header.stamp).nanoseconds();
        pc->clear();
        const size_t n = pc_in.size();
        pc_out->points.resize(n);
        std::vector<uint8_t> valid(n, 0);
        #pragma omp parallel for num_threads(num_threads_)
        for (size_t i = 0; i < n; i++) {
            const PointType& pt = pc_in.points[i];
            int64_t t_ns = int64_t(pt.intensity * float(1e6)) + time_begin;
            if (t_ns >= spl->minTimeNs() && t_ns <= spl->maxTimeNs()) {
                pc_out->points[i] = transformPoint(t_ns, spl, pt);
                valid[i] = 1;
            }
        }
        // Compact: remove slots where the timestamp was out of range
        size_t write_idx = 0;
        for (size_t i = 0; i < n; i++) {
            if (valid[i]) {
                pc_out->points[write_idx++] = pc_out->points[i];
            }
        }
        pc_out->points.resize(write_idx);
    }

  protected:
    rclcpp::QoS lidar_qos;
    rclcpp::SubscriptionOptions sub_opt;    
    Eigen::aligned_deque<typename pcl::PointCloud<PointType>> pc_L_buff;
    typename pcl::PointCloud<PointType>::Ptr pc_last;
    typename pcl::PointCloud<PointType>::Ptr pc_last_ds;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_each_scan;
    std::string frame_id = "base_link";
    std::string map_id = "map";
    int num_threads_ = 5;
    
    // TF transformation
    rclcpp::Node::SharedPtr node_handle_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_lidar_transform_;
    bool have_imu_transform_;
    Eigen::Affine3d lidar_to_baselink_;
    Eigen::Affine3d imu_to_baselink_;

    typename pcl::PointCloud<PointType>::Ptr pc;
};


class OusterBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  OusterBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config, 
             rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr) 
      : MappingBase<pcl::PointXYZINormal>(nh, lidar_config, sensor_cb)
    {
        pc_subscription_ouster = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->lidar.topic, lidar_qos, std::bind(&OusterBuff::ousterLidarCallback, this, std::placeholders::_1), sub_opt);
        double lidar_time_offset = CommonUtils::readParam<double>(nh->get_node_parameters_interface(), "lidar_time_offset", 0.0);
        time_offset = 1e9*lidar_time_offset;
    }

    void ousterLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ouster_msg_in)
    {
        // Guard against negative timestamps (sim-time messages)
        int64_t stamp_ns = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds();
        if (stamp_ns < time_offset) return;

        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(ouster_msg_in->header.frame_id)) return;

        this->pc_last->clear();
        pcl::PointCloud<ouster_ros::Point>::Ptr pc_last_ouster(new pcl::PointCloud<ouster_ros::Point>());
        pcl::fromROSMsg(*ouster_msg_in, *pc_last_ouster);
        size_t plsize = pc_last_ouster->size();
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (uint i = 1; i < plsize; i++) {
            pt.x = pc_last_ouster->points[i].x;
            pt.y = pc_last_ouster->points[i].y;
            pt.z = pc_last_ouster->points[i].z;
            pt.intensity = float (pc_last_ouster->points[i].t) / float (1e6); // unit: ms
            pt.curvature = 0.1 * pc_last_ouster->points[i].intensity;

            if (pt.intensity >= 0) {
                this->pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds() - time_offset;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;

        // Transform point cloud to body frame
        pcl::transformPointCloud(*this->pc_last, *this->pc_last, lidar_to_baselink_);

        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds() - time_offset;
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Cap cloud buffer to prevent OOM on processing stalls
            while (this->pc_L_buff.size() >= 5) { this->pc_L_buff.pop_front(); }
            this->pc_L_buff.push_back(*pc_last_ds);
        }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_ouster;
    int64_t time_offset = 0;
};

class Mid70AviaBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  Mid70AviaBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config,
                rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr)
      : MappingBase<pcl::PointXYZINormal>(nh, lidar_config, sensor_cb)
    {
        pc_subscription_livox = nh->create_subscription<livox_ros_driver::msg::CustomMsg>(
            this->lidar.topic, lidar_qos, std::bind(&Mid70AviaBuff::livoxLidarCallback, this, std::placeholders::_1), sub_opt);
    }

    void livoxLidarCallback(const livox_ros_driver::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(livox_msg_in->header.frame_id)) return;
        
        this->pc_last->clear();
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (int i = 1; i < plsize; i++) {
            if ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) {
                pt.x = livox_msg_in->points[i].x;
                pt.y = livox_msg_in->points[i].y;
                pt.z = livox_msg_in->points[i].z;
                pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms
                pt.curvature = livox_msg_in->points[i].reflectivity;
                pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;
 
        // Transform point cloud to body frame
        pcl::transformPointCloud(*this->pc_last, *this->pc_last, lidar_to_baselink_);

        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Cap cloud buffer to prevent OOM on processing stalls
            while (this->pc_L_buff.size() >= 5) { this->pc_L_buff.pop_front(); }
            this->pc_L_buff.push_back(*pc_last_ds);
        }
    }

  private:
    rclcpp::Subscription<livox_ros_driver::msg::CustomMsg>::SharedPtr pc_subscription_livox;
};

class HAP360Buff : public MappingBase<pcl::PointXYZINormal>
{
public:
    HAP360Buff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config,
               rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr)
        : MappingBase<pcl::PointXYZINormal>(nh, lidar_config, sensor_cb)
    {
        pc_subscription_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            this->lidar.topic, lidar_qos, std::bind(&HAP360Buff::livoxLidarCallback, this, std::placeholders::_1), sub_opt);
    }

    void livoxLidarCallback(livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(livox_msg_in->header.frame_id)) return;
        
        this->pc_last->clear();
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (int i = 1; i < plsize; i++) {
            if ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) {
                pt.x = livox_msg_in->points[i].x;
                pt.y = livox_msg_in->points[i].y;
                pt.z = livox_msg_in->points[i].z;
                pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms
                pt.curvature = livox_msg_in->points[i].reflectivity;
                pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;

        // Transform point cloud to body frame
        pcl::transformPointCloud(*this->pc_last, *this->pc_last, lidar_to_baselink_);

        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Cap cloud buffer to prevent OOM on processing stalls
            while (this->pc_L_buff.size() >= 5) { this->pc_L_buff.pop_front(); }
            this->pc_L_buff.push_back(*pc_last_ds);
        }
    }

  private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr pc_subscription_livox;
};

class AviaRespleBuff : public MappingBase<pcl::PointXYZINormal>
{
public:
    AviaRespleBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config,
                   rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr)
        : MappingBase<pcl::PointXYZINormal>(nh, lidar_config, sensor_cb)
    {
        pc_subscription_livox = nh->create_subscription<livox_interfaces::msg::CustomMsg>(
            this->lidar.topic, lidar_qos, std::bind(&AviaRespleBuff::livoxLidarCallback, this, std::placeholders::_1), sub_opt);
    }

    void livoxLidarCallback(livox_interfaces::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(livox_msg_in->header.frame_id)) return;
                
        this->pc_last->clear();
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        for (int i = 1; i < plsize; i++) {
            if ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) {
                pt.x = livox_msg_in->points[i].x;
                pt.y = livox_msg_in->points[i].y;
                pt.z = livox_msg_in->points[i].z;
                pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms
                pt.curvature = livox_msg_in->points[i].reflectivity;
                pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;

        // Transform point cloud to body frame
        pcl::transformPointCloud(*this->pc_last, *this->pc_last, lidar_to_baselink_);

        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Cap cloud buffer to prevent OOM on processing stalls
            while (this->pc_L_buff.size() >= 5) { this->pc_L_buff.pop_front(); }
            this->pc_L_buff.push_back(*pc_last_ds);
        }
    }

  private:
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr pc_subscription_livox;
};

class HesaiBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  HesaiBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config,
            rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr)
      : MappingBase<pcl::PointXYZINormal>(nh, lidar_config, sensor_cb)
    {
        pc_subscription_hesai = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->lidar.topic, lidar_qos, std::bind(&HesaiBuff::hesaiLidarCallback, this, std::placeholders::_1), sub_opt);
    }

    void hesaiLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr hesai_msg_in)
    {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(hesai_msg_in->header.frame_id)) return;
                
        this->pc_last->clear();
        pcl::PointCloud<hesai_ros::Point>::Ptr pc_last_hesai(new pcl::PointCloud<hesai_ros::Point>());
        pcl::fromROSMsg(*hesai_msg_in, *pc_last_hesai);
        size_t plsize = pc_last_hesai->size();
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(hesai_msg_in->header.stamp);
        pcl::PointXYZINormal pt;
        for (uint i = 0; i < plsize; i++) {
            pt.x = pc_last_hesai->points[i].x;
            pt.y = pc_last_hesai->points[i].y;
            pt.z = pc_last_hesai->points[i].z;
            double timestamp_s;
            double timestamp_ns = std::modf(pc_last_hesai->points[i].timestamp, &timestamp_s);
            rclcpp::Time timestamp_ros(static_cast<uint32_t>(timestamp_s), static_cast<uint32_t>(timestamp_ns * 1.0e9),
                rcl_clock_type_t::RCL_ROS_TIME);
            pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3;
            pt.curvature = pc_last_hesai->points[i].intensity;

            if (pt.intensity >= 0) {
                this->pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(hesai_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;

        // Transform point cloud to body frame
        pcl::transformPointCloud(*this->pc_last, *this->pc_last, lidar_to_baselink_);

        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(hesai_msg_in->header.stamp).nanoseconds();
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Cap cloud buffer to prevent OOM on processing stalls
            while (this->pc_L_buff.size() >= 5) { this->pc_L_buff.pop_front(); }
            this->pc_L_buff.push_back(*pc_last_ds);
        }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_hesai;
};

class Mid360BoxiBuff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  Mid360BoxiBuff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config,
                 rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr)
      : MappingBase<pcl::PointXYZINormal>(nh, lidar_config, sensor_cb)
    {
        pc_subscription_mid360 = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->lidar.topic, lidar_qos, std::bind(&Mid360BoxiBuff::mid360BoxiCallback, this, std::placeholders::_1), sub_opt);
    }

    void mid360BoxiCallback(const sensor_msgs::msg::PointCloud2::SharedPtr livox_msg_in)
    {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(livox_msg_in->header.frame_id)) return;
        
        this->pc_last->clear();
        pcl::PointCloud<livox_mid360_boxi::Point>::Ptr pc_last_livox(new pcl::PointCloud<livox_mid360_boxi::Point>());
        pcl::fromROSMsg(*livox_msg_in, *pc_last_livox);
        size_t plsize = pc_last_livox->size();
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(livox_msg_in->header.stamp);
        pcl::PointXYZINormal pt;
        for (uint i = 0; i < plsize; i++) {
            pt.x = pc_last_livox->points[i].x;
            pt.y = pc_last_livox->points[i].y;
            pt.z = pc_last_livox->points[i].z;
            rclcpp::Time timestamp_ros(static_cast<int64_t>(pc_last_livox->points[i].timestamp),
                rcl_clock_type_t::RCL_ROS_TIME);
            pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3;
            pt.curvature = pc_last_livox->points[i].intensity;

            if (pt.intensity >= 0) {
                this->pc_last->points.push_back(pt);
            }
        }
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;

        // Transform point cloud to body frame
        pcl::transformPointCloud(*this->pc_last, *this->pc_last, lidar_to_baselink_);

        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Cap cloud buffer to prevent OOM on processing stalls
            while (this->pc_L_buff.size() >= 5) { this->pc_L_buff.pop_front(); }
            this->pc_L_buff.push_back(*pc_last_ds);
        }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_mid360;
};

class Mapping : public rclcpp_lifecycle::LifecycleNode
{

public:

Mapping(const rclcpp::NodeOptions& options, std::vector<MappingBase<pcl::PointXYZINormal>*>& mappings)
    : rclcpp_lifecycle::LifecycleNode("Mapping", 
          rclcpp::NodeOptions(options).use_intra_process_comms(true)),
      processing_active_(false),
      vis_maps(mappings)
    {
        RCLCPP_INFO(this->get_logger(), "Mapping LifecycleNode created (unconfigured state)");
        spl_window_st_ns = 0;
        opt_old_path.header.frame_id = map_id;
    }
    
    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring Mapping...");

        // Initialize TF buffer and listener
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Read frame ID parameters
        frame_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "frame_id", "base_link");
        odom_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "odom/frame_id", "odom");
        map_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "map/frame_id", "map");

        publish_tf = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "map/publish_tf", true);
        invert_tf  = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "map/invert_tf", false);        
    
        std::vector<double> cov_varp = CommonUtils::readParam<std::vector<double>>(this->get_node_parameters_interface(), "cov_pose", {0.2, 0.2, 0.2, 0.1, 0.1, 0.1});
        cov_pose << cov_varp.at(0), cov_varp.at(1), cov_varp.at(2), cov_varp.at(3), cov_varp.at(4), cov_varp.at(5);        

        std::vector<double> cov_vart = CommonUtils::readParam<std::vector<double>>(this->get_node_parameters_interface(), "cov_twist", {0.2, 0.2, 0.2, 0.1, 0.1, 0.1});
        cov_twist << cov_vart.at(0), cov_vart.at(1), cov_vart.at(2), cov_vart.at(3), cov_vart.at(4), cov_vart.at(5);        


        // Create publishers (inactive until activated)
        pub_path = this->create_publisher<nav_msgs::msg::Path>("traj_path",
            rclcpp::QoS(20).reliable());
        pub_knots = this->create_publisher<sensor_msgs::msg::PointCloud>("active_control_points",
            rclcpp::QoS(20).reliable());
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odometry",
            rclcpp::QoS(500).reliable());
        br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "Mapping configured successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Activating Mapping...");
        
        // Activate publishers
        pub_path->on_activate();
        pub_knots->on_activate();
        pub_odom->on_activate();
        
        // Create subscriptions
        auto reliable_qos = rclcpp::QoS(100).reliable();
        auto large_reliable_qos = rclcpp::QoS(10000).reliable();
        sub_start = this->create_subscription<std_msgs::msg::Int64>("start_time", reliable_qos, 
            std::bind(&Mapping::startCallBack, this, std::placeholders::_1));
        sub_est = this->create_subscription<estimate_msgs::msg::Estimate>("est_window", large_reliable_qos, 
            std::bind(&Mapping::getEstCallback, this, std::placeholders::_1));
        
        // Start processing thread
        processing_active_ = true;
        processing_thread_ = std::thread(&Mapping::process, this);
        
        RCLCPP_INFO(this->get_logger(), "Mapping activated successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating Mapping...");
        
        // Stop processing thread
        processing_active_ = false;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        // Deactivate publishers
        pub_path->on_deactivate();
        pub_knots->on_deactivate();
        pub_odom->on_deactivate();
        
        // Reset subscriptions
        sub_start.reset();
        sub_est.reset();

        // Clear init flag so a re-activation starts in the unitialized state.
        // Worker is joined; safe to write without ordering concerns.
        if_init_succeed.store(false, std::memory_order_relaxed);

        // Drain in-flight callbacks before on_cleanup tears down related state.
        // Mirror of the RESPLE on_deactivate barrier; same rationale (rclcpp
        // Subscription::reset() does not synchronize with executor-dispatched
        // callbacks).
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(this->get_logger(), "Mapping deactivated successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up Mapping...");
        
        // Reset publishers
        pub_path.reset();
        pub_knots.reset();
        pub_odom.reset();
        br.reset();
        
        // Clear data
        opt_old_path.poses.clear();
        path_t_ns_ = 0;
        if_init_succeed = false;
        
        RCLCPP_INFO(this->get_logger(), "Mapping cleaned up successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Mapping...");
        
        // Ensure processing thread is stopped
        processing_active_ = false;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        RCLCPP_INFO(this->get_logger(), "Mapping shutdown complete");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // RAII helper: locks every per-map mtx on construction, releases them on
    // destruction (in reverse order). Replaces the prior lock_mappings() /
    // unlock_mappings() pair, which leaked locks if anything between them
    // threw — next iteration would deadlock on its own mtx_pc.acquire().
    class ScopedMappingsLock {
    public:
        explicit ScopedMappingsLock(std::vector<MappingBase<pcl::PointXYZINormal>*>& maps)
            : maps_(maps)
        {
            for (auto* m : maps_) m->mtx.lock();
        }
        ~ScopedMappingsLock() {
            for (auto it = maps_.rbegin(); it != maps_.rend(); ++it) (*it)->mtx.unlock();
        }
        ScopedMappingsLock(const ScopedMappingsLock&) = delete;
        ScopedMappingsLock& operator=(const ScopedMappingsLock&) = delete;
    private:
        std::vector<MappingBase<pcl::PointXYZINormal>*>& maps_;
    };

    void process() {
        // RCLCPP_INFO(this->get_logger(), "process");
        rclcpp::Rate rate(10);
        int64_t num_knot = 0;
        while (processing_active_ && rclcpp::ok()) {
            // Swap pending spline to active (lock-free check then mutex swap)
            if (spline_pending_ready_.load()) {
                std::lock_guard<std::mutex> lock(m_spline);
                if (spline_pending_ready_.load()) {
                    ScopedMappingsLock maps_lock(vis_maps);
                    spl_window_st_ns = spl_window_st_ns_pending_;
                    spline_active_.setTimeIntervalNs(spline_pending_.getKnotTimeIntervalNs());
                    spline_active_.updateKnots(&spline_pending_);
                    spline_pending_ready_.store(false);
                }
            }
            // Acquire-load on if_init_succeed so the startCallBack init()
            // writes (release-store on the flag) are visible before we read
            // spline_active_'s state. After this gate, all subsequent reads
            // happen on the same worker thread as the swap above, so they
            // are single-threaded with respect to spline_active_ mutation —
            // no further m_spline coverage needed for steady state.
            if (if_init_succeed.load(std::memory_order_acquire) && spline_active_.numKnots() > num_knot) {
                ScopedMappingsLock maps_lock(vis_maps);
                publishPath();
                displayControlPoints();
                pubOdom();
                num_knot = spline_active_.numKnots();
            }
            if (!if_init_succeed.load(std::memory_order_acquire)) {
                if (rclcpp::ok()) {
                    rate.sleep();
                }
                continue;
            }
            for (const auto vis_map : vis_maps) {
                vis_map->processScan(&spline_active_, spl_window_st_ns);
            }
        }
    }

private:
    std::string node_name = "Mapping";
    int64_t spl_window_st_ns;
    // Double-buffered spline: getEstCallback writes to spline_pending_,
    // process() copies pending to spline_active_ under m_spline mutex.
    // This eliminates the race between the callback thread and process thread.
    SplineState spline_active_;
    SplineState spline_pending_;
    std::atomic<bool> spline_pending_ready_{false};
    int64_t spl_window_st_ns_pending_ = 0;
    rclcpp::Subscription<estimate_msgs::msg::Estimate>::SharedPtr sub_est;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_start;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    nav_msgs::msg::Path opt_old_path;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_knots;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Lifecycle management
    std::atomic<bool> processing_active_;
    std::thread processing_thread_;
    std::vector<MappingBase<pcl::PointXYZINormal>*> vis_maps;
    std::string frame_id;
    std::string odom_id;
    std::string map_id;
    Eigen::Vector<double, 6> cov_pose;
    Eigen::Vector<double, 6> cov_twist;
    bool publish_tf, invert_tf;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    // Atomic so process() (worker thread) reads the flag without racing with
    // startCallBack (executor thread) writing it. The store in startCallBack
    // happens AFTER spline_active_.init(); using release ordering on the
    // store + acquire on the loads ensures the worker observes the fully
    // initialized spline_active_ when it sees the flag true.
    std::atomic<bool> if_init_succeed{false};
    int64_t path_t_ns_ = 0;
    std::mutex m_spline;

    void displayControlPoints()
    {
        if (spline_active_.numKnots() < 4) { return; }
        if (spline_active_.maxTimeNs() < 0) { return; }
        sensor_msgs::msg::PointCloud points_msg;
        points_msg.header.frame_id = map_id;
        points_msg.header.stamp = rclcpp::Time(spline_active_.maxTimeNs());
        for (int64_t i = spline_active_.numKnots() - 4; i < spline_active_.numKnots(); i++) {
            points_msg.points.push_back(CommonUtils::getPointMsg(spline_active_.getKnotPos(i)));
        }
        pub_knots->publish(points_msg);
    }

    void getEstCallback(const estimate_msgs::msg::Estimate::SharedPtr est_msg)
    {
        if (!if_init_succeed) {
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "getEstCallback");

        estimate_msgs::msg::Spline spline_msg = est_msg->spline;
        SplineState spline_w;

        spline_w.init(spline_msg.dt, 0, spline_msg.start_t, spline_msg.start_idx);
        for(const auto& knot : spline_msg.knots) {
            Eigen::Vector3d pos(knot.position.x, knot.position.y, knot.position.z);
            Eigen::Vector3d quat_del(knot.orientation_del.x, knot.orientation_del.y, knot.orientation_del.z);
            spline_w.addOneStateKnot(pos, quat_del);
        }
        Eigen::Quaterniond q_idle0 = Eigen::Quaterniond(spline_msg.start_q.w, spline_msg.start_q.x, spline_msg.start_q.y, spline_msg.start_q.z);
        for (int i = 0; i < 3 && i < (int)spline_msg.idles.size(); i++) {
            estimate_msgs::msg::Knot idle = spline_msg.idles[i];
            Eigen::Vector3d t_idle(idle.position.x, idle.position.y, idle.position.z);
            Eigen::Vector3d quat_idle(idle.orientation_del.x, idle.orientation_del.y, idle.orientation_del.z);
            spline_w.setIdles(i, t_idle, quat_idle, q_idle0);
        }
        // Write to pending buffer — process() will swap to active under mutex.
        //
        // Move-assign (don't merge): spline_pending_ must hold THIS window's
        // knots/start_i intact so process()'s spline_active_.updateKnots()
        // sees the correct global indexing offset. The previous code called
        // spline_pending_.updateKnots(&spline_w), which:
        //   (a) read spline_pending_.num_knot before it was ever initialized
        //       (default ctor leaves int64_t indeterminate → loop bound was
        //       garbage on first call → setOneStateKnot writes out of bounds
        //       → SIGSEGV in Eigen Vector3d copy);
        //   (b) accumulated knots across callbacks, so spline_pending_ grew
        //       unbounded and start_i never matched the latest window.
        {
            std::lock_guard<std::mutex> lock(m_spline);
            spl_window_st_ns_pending_ = spline_msg.start_t - spline_msg.dt;
            spline_pending_ = std::move(spline_w);
            spline_pending_ready_.store(true);
        }
    }

    void pubOdom()
    {
        // RCLCPP_INFO(this->get_logger(), "pubOdom");    
        if (opt_old_path.poses.size() < 2) {
            return;
        }
        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::PoseStamped odom_pose_current = opt_old_path.poses.rbegin()[0];
        geometry_msgs::msg::PoseStamped odom_pose_last = opt_old_path.poses.rbegin()[1];
        
        // Use the latest spline time for better sync with current_scan
        // rclcpp::Time current_time = rclcpp::Time(spline_active_.maxTimeNs());
        // odom_msg.header.stamp = current_time;
        
        odom_msg.header.stamp = rclcpp::Time(odom_pose_current.header.stamp);
        odom_msg.header.frame_id = map_id;

        odom_msg.child_frame_id = frame_id;
        odom_msg.pose.pose = odom_pose_current.pose;
        
        // Covariance
        odom_msg.pose.covariance[0]  =  cov_pose[0];
        odom_msg.pose.covariance[7]  =  cov_pose[1];
        odom_msg.pose.covariance[14] =  cov_pose[2];
        odom_msg.pose.covariance[21] =  cov_pose[3];
        odom_msg.pose.covariance[28] =  cov_pose[4];
        odom_msg.pose.covariance[35] =  cov_pose[5];
        
        // Quaternions for current and last poses
        Eigen::Quaterniond q_cur(
            odom_pose_current.pose.orientation.w,
            odom_pose_current.pose.orientation.x,
            odom_pose_current.pose.orientation.y,
            odom_pose_current.pose.orientation.z);
        Eigen::Quaterniond q_last(
            odom_pose_last.pose.orientation.w,
            odom_pose_last.pose.orientation.x,
            odom_pose_last.pose.orientation.y,
            odom_pose_last.pose.orientation.z);

        double dt = (rclcpp::Time(odom_pose_current.header.stamp) - rclcpp::Time(odom_pose_last.header.stamp)).seconds();
        if(dt < 1e-10) {
            RCLCPP_INFO(this->get_logger(), "dt too small!");
        } else{
            // Linear velocity in map frame, then rotate to body frame (REP-105)
            Eigen::Vector3d v_map(
                (odom_pose_current.pose.position.x - odom_pose_last.pose.position.x)/dt,
                (odom_pose_current.pose.position.y - odom_pose_last.pose.position.y)/dt,
                (odom_pose_current.pose.position.z - odom_pose_last.pose.position.z)/dt);
            Eigen::Vector3d v_body = q_cur.inverse() * v_map;
            odom_msg.twist.twist.linear.x = v_body.x();
            odom_msg.twist.twist.linear.y = v_body.y();
            odom_msg.twist.twist.linear.z = v_body.z();

            // Angular velocity via quaternion delta (avoids RPY gimbal issues)
            Eigen::Quaterniond q_delta = q_last.inverse() * q_cur;
            // Ensure shortest path
            if (q_delta.w() < 0.0) {
                q_delta.coeffs() = -q_delta.coeffs();
            }
            Eigen::AngleAxisd aa(q_delta);
            Eigen::Vector3d w_body = aa.axis() * aa.angle() / dt;
            odom_msg.twist.twist.angular.x = w_body.x();
            odom_msg.twist.twist.angular.y = w_body.y();
            odom_msg.twist.twist.angular.z = w_body.z();
        }

        odom_msg.twist.covariance[0]  =  cov_twist[0];
        odom_msg.twist.covariance[7]  =  cov_twist[1];
        odom_msg.twist.covariance[14] =  cov_twist[2];
        odom_msg.twist.covariance[21] =  cov_twist[3];
        odom_msg.twist.covariance[28] =  cov_twist[4];
        odom_msg.twist.covariance[35] =  cov_twist[5];        

        pub_odom->publish(odom_msg);      
        
        // Publish map transforms        
        if(publish_tf)
        {
            // Calculate desired frame to map transform            
            geometry_msgs::msg::TransformStamped baselink_to_map;
            baselink_to_map.header.stamp = odom_msg.header.stamp;
            baselink_to_map.header.frame_id = map_id;
            baselink_to_map.child_frame_id = frame_id;
            baselink_to_map.transform.translation.x = odom_pose_current.pose.position.x;
            baselink_to_map.transform.translation.y = odom_pose_current.pose.position.y;
            baselink_to_map.transform.translation.z = odom_pose_current.pose.position.z;
            baselink_to_map.transform.rotation = odom_pose_current.pose.orientation;

            // Get frame to odom transform
            geometry_msgs::msg::TransformStamped odom_to_baselink;
            bool got_odom_transform = false;
            try {
                if (tf_buffer->canTransform(this->frame_id, this->odom_id,
                                            odom_msg.header.stamp, rclcpp::Duration::from_seconds(0.1))) {
                    odom_to_baselink = tf_buffer->lookupTransform(this->frame_id, this->odom_id, odom_msg.header.stamp);
                    got_odom_transform = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                        "[Mapping] Waiting for odom transform: %s -> %s",
                                        this->frame_id.c_str(), this->odom_id.c_str());
                }
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "[Mapping] LiDAR transform exception: %s", ex.what());
            }

            // Subtract baselink_to_odom transform from baselink_to_map transform to get odom_to_map transform
            if (got_odom_transform) {
                tf2::Transform odom_to_baselink_tf, baselink_to_map_tf, odom_to_map_tf;
                tf2::fromMsg(odom_to_baselink.transform, odom_to_baselink_tf);
                tf2::fromMsg(baselink_to_map.transform, baselink_to_map_tf);
                odom_to_map_tf  = baselink_to_map_tf * odom_to_baselink_tf;
                if(invert_tf){
                    odom_to_map_tf = odom_to_map_tf.inverse();
                }
                geometry_msgs::msg::TransformStamped odom_to_map;
                tf2::toMsg(odom_to_map_tf, odom_to_map.transform);
                odom_to_map.header.stamp = odom_msg.header.stamp;
                if(invert_tf){
                    odom_to_map.header.frame_id = odom_id;
                    odom_to_map.child_frame_id  = map_id;
                } else {
                    odom_to_map.header.frame_id = map_id;
                    odom_to_map.child_frame_id  = odom_id;
                }
                br->sendTransform(odom_to_map);
            }
        }
    }

    void startCallBack(const std_msgs::msg::Int64::SharedPtr start_time_msg)
    {
        // Init under m_spline so a concurrent process() iteration that's
        // mid-swap (lock_guard<mutex>(m_spline)) can't observe a partially
        // constructed spline_active_. The release-store on if_init_succeed
        // pairs with process()'s acquire-load to guarantee the init writes
        // are visible.
        int64_t bag_start_time = start_time_msg->data;
        {
            std::lock_guard<std::mutex> lock(m_spline);
            spline_active_.init(1, 0, bag_start_time, 0);  // dt=1 placeholder; overridden by getEstCallback
        }
        if_init_succeed.store(true, std::memory_order_release);
    }

    void publishPath() {
        // RCLCPP_INFO(this->get_logger(), "publishPath");
        if (!if_init_succeed || spline_active_.numKnots() <= 4) {
            return;
        }
        if (spline_active_.maxTimeNs() < 0 || spline_active_.minTimeNs() < 0) {
            return;
        }
        if (path_t_ns_ == 0) {
            path_t_ns_ = spline_active_.minTimeNs();
        }
        while (path_t_ns_ < std::min(spl_window_st_ns, spline_active_.maxTimeNs())) {
            Eigen::Quaterniond orient_interp;
            Eigen::Vector3d t_interp = spline_active_.itpPosition(path_t_ns_);
            spline_active_.itpQuaternion(path_t_ns_, &orient_interp);
            opt_old_path.poses.push_back(CommonUtils::pose2msg(map_id, path_t_ns_, t_interp, orient_interp));
            path_t_ns_ += 1e8;
        }
        // Cap path length to prevent OOM on long runs
        if (opt_old_path.poses.size() > 10000) {
            opt_old_path.poses.erase(opt_old_path.poses.begin(),
                opt_old_path.poses.begin() + (opt_old_path.poses.size() - 10000));
        }
        opt_old_path.header.frame_id = map_id;
        opt_old_path.header.stamp = rclcpp::Time(spline_active_.maxTimeNs());
        pub_path->publish(opt_old_path);
    }

};

// Crash handler: prints a backtrace to stderr on fatal signals, then restores
// the default handler and re-raises. Mirror of the handler in RESPLE.cpp.
// See that file's comment block for rationale (added 2026-04-21 to debug a
// deterministic SIGSEGV after gravity alignment).
static void mappingCrashHandler(int sig)
{
    constexpr int kMaxFrames = 64;
    void *addrs[kMaxFrames];
    int n = backtrace(addrs, kMaxFrames);
    const char *name = (sig == SIGSEGV) ? "SIGSEGV" :
                       (sig == SIGABRT) ? "SIGABRT" :
                       (sig == SIGBUS)  ? "SIGBUS"  :
                       (sig == SIGFPE)  ? "SIGFPE"  :
                       (sig == SIGILL)  ? "SIGILL"  : "unknown";
    dprintf(STDERR_FILENO, "\n=== Mapping crash handler: caught %s (%d), %d frames ===\n",
            name, sig, n);
    backtrace_symbols_fd(addrs, n, STDERR_FILENO);
    dprintf(STDERR_FILENO, "=== end Mapping backtrace ===\n");
    signal(sig, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv) {
    signal(SIGSEGV, mappingCrashHandler);
    signal(SIGABRT, mappingCrashHandler);
    signal(SIGBUS,  mappingCrashHandler);
    signal(SIGFPE,  mappingCrashHandler);
    signal(SIGILL,  mappingCrashHandler);

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    rclcpp::init(argc, argv);
    
    // Create temporary node for parameter loading (use unique name)
    rclcpp::NodeOptions temp_options;
    temp_options.arguments({"--ros-args", "-r", "__node:=MappingInit"});
    auto temp_nh = rclcpp::Node::make_shared("MappingInit", temp_options);
    
    std::vector<LidarConfig> lidars;
    auto lidar_names = temp_nh->declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>());
    assert(temp_nh->get_parameter({"lidars"}, lidar_names));
    if (lidar_names.empty()) {
        lidars.emplace_back(temp_nh->get_node_parameters_interface(), "");
    } else {
        for (const auto& lidar_name : lidar_names) {
            lidars.emplace_back(temp_nh->get_node_parameters_interface(), lidar_name + ".");
        }
    }
    // Create callback group for sensor processing
    auto sensor_cb_group = temp_nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    std::vector<MappingBase<pcl::PointXYZINormal>*> buffs;
    for (const auto& lidar : lidars) {
        if (!lidar.type.compare("Ouster")) {
            buffs.push_back(new OusterBuff(temp_nh, lidar, sensor_cb_group));
        } else if (!lidar.type.compare("Mid70Avia")) {
            buffs.push_back(new Mid70AviaBuff(temp_nh, lidar, sensor_cb_group));
        } else if (!lidar.type.compare("HAP360")) {
            buffs.push_back(new HAP360Buff(temp_nh, lidar, sensor_cb_group));
        } else if (!lidar.type.compare("AviaResple")) {
            buffs.push_back(new AviaRespleBuff(temp_nh, lidar, sensor_cb_group));
        } else if (!lidar.type.compare("Hesai")) {
            buffs.push_back(new HesaiBuff(temp_nh, lidar, sensor_cb_group));
        } else if (!lidar.type.compare("Mid360Boxi")) {
            buffs.push_back(new Mid360BoxiBuff(temp_nh, lidar, sensor_cb_group));
        } else {
            exit(1);
        }
    }
    
    // Lifecycle node initialization
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Mapping>(options, buffs);
    RCLCPP_INFO(node->get_logger(), "Mapping LifecycleNode created");
    
    // Transition to configured state
    node->configure();
    RCLCPP_INFO(node->get_logger(), "Mapping configured");
    
    // Transition to active state (starts processing)
    node->activate();
    RCLCPP_INFO(node->get_logger(), "Mapping activated - processing started");
    
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.add_node(temp_nh);
    exec.spin();
    
    // Cleanup on shutdown (only if context still valid)
    if (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Gracefully shutting down Mapping...");
        node->deactivate();
        node->cleanup();
        node->shutdown();
    } else {
        // Context already shut down (Ctrl+C), just stop processing
        RCLCPP_WARN(node->get_logger(), "Context invalid, forcing shutdown...");
        // Manually trigger shutdown to stop thread
        node->on_shutdown(node->get_current_state());
    }
    exec.remove_node(node->get_node_base_interface());
    exec.remove_node(temp_nh);
    
    // Cleanup buffs
    for (auto buff : buffs) {
        delete buff;
    }
    
    rclcpp::shutdown();
}
