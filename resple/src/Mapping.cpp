// 2026-05-01 Eigen aligned-allocator ABI fix — MUST be the first thing in this
// TU, before any header that pulls in Eigen. See RESPLE.cpp top-of-file block
// and HARDENING.md Phase 1.6 for context.
#ifndef EIGEN_MALLOC_ALREADY_ALIGNED
#define EIGEN_MALLOC_ALREADY_ALIGNED 1
#endif
#ifndef EIGEN_DEFAULT_ALIGN_BYTES
#define EIGEN_DEFAULT_ALIGN_BYTES 16
#endif
#ifndef EIGEN_MAX_ALIGN_BYTES
#define EIGEN_MAX_ALIGN_BYTES 16
#endif

#include <thread>
#include <iostream>
#include <queue>
#include <string>
#include <cmath>
#include <atomic>
#include <chrono>
#include <future>
#include <cstdlib>
#include <cstdio>
#include <csignal>
#include <execinfo.h>
#include <unistd.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "utils/common_utils.h"
#include "SplineState.h"
#include "utils/point_cloud_ingest.h"
#include "utils/tf_ownership.h"

template<typename PointType>
class MappingBase
{
  public:

    std::mutex mtx;       // protects pc_L_buff (shared with the worker thread)
    // Serializes the sensor callback body. The lidar subs share a
    // MutuallyExclusive callback group, but under the MultiThreadedExecutor TSan
    // observes no happens-before between consecutive callback dispatches, so the
    // shared scratch members (pc_last / pc_last_ds / ds_filter_each_scan /
    // transform state) raced callback-vs-callback (HARDENING Phase 6.1). This
    // lock makes the serialization explicit and TSan-visible.
    std::mutex cb_mtx_;
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
        // Same OpenMP clamp as the RESPLE node (overload hardening 2026-07-07):
        // configs written for big machines must not oversubscribe a small one.
        {
            const unsigned hc = std::thread::hardware_concurrency();
            if (hc > 0) {
                const int cap = std::max(1, static_cast<int>(hc) - 2);
                if (num_threads_ > cap) {
                    RCLCPP_WARN(nh->get_logger(),
                        "num_threads=%d exceeds this machine's budget (%u cores - 2); "
                        "clamping to %d", num_threads_, hc, cap);
                    num_threads_ = cap;
                }
            }
        }
        // Map lag in knots (see processScan). Default 8 = the convergence
        // horizon plus margin: the IEKF only updates the last 4 RCPs and
        // est_window only resends the last 5 knots, so a knot is FINAL once
        // ~4 behind the edge; cubic interpolation at scan time t reads knots
        // up to idx(t)+2, so every knot a scan touches is final once the edge
        // is >= 6 knots past it. Beyond ~8 the lag buys zero further
        // refinement — only latency (80 ms at knot_hz 100, well inside the
        // 0.5 s map-latency budget). 0 restores the bleeding-edge behavior.
        deskew_lag_knots_ = CommonUtils::readParam<int>(nh->get_node_parameters_interface(), "map_deskew_lag_knots", 8);
        if (deskew_lag_knots_ < 0) {
            RCLCPP_WARN(nh->get_logger(),
                "map_deskew_lag_knots=%d is negative; using 0 (no lag)", deskew_lag_knots_);
            deskew_lag_knots_ = 0;
        }
        // Overload hardening (2026-07-07): the per-sensor scan-buffer cap was
        // hardcoded 200, duplicated in every callback, and dropped silently
        // (permanent map holes with no trace). Now parameterized (default 200
        // = previous behaviour) and counted — see pushScanCapped().
        max_scan_buffer_ = CommonUtils::readParam<int>(nh->get_node_parameters_interface(), "map/max_scan_buffer", 200);
        if (max_scan_buffer_ < 1) {
            RCLCPP_WARN(nh->get_logger(),
                "map/max_scan_buffer=%d is < 1; using 1", max_scan_buffer_);
            max_scan_buffer_ = 1;
        }
        // Batch publish: accumulate transformed scans and publish /global_map
        // at most every N ms (0 = per-scan, previous behaviour). Batching
        // never drops content — see processScan.
        publish_min_interval_ms_ = CommonUtils::readParam<int>(nh->get_node_parameters_interface(), "map/publish_min_interval_ms", 0);
        if (publish_min_interval_ms_ > 0) {
            RCLCPP_INFO(nh->get_logger(),
                "[Mapping] /global_map batched: publishing at most every %d ms",
                publish_min_interval_ms_);
        }

        RCLCPP_INFO(nh->get_logger(), "Frame IDs -  map: %s, body: %s", 
                    map_id.c_str(), frame_id.c_str());        

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        have_lidar_transform_ = false;
        have_imu_transform_ = false;
        lidar_to_baselink_ = Eigen::Affine3d::Identity();
        imu_to_baselink_ = Eigen::Affine3d::Identity();
        baselink_to_imu_ = Eigen::Affine3d::Identity();
        // Same extrinsic convention switch as the RESPLE node (findings
        // #14/#22): the dataset launches pass tf_extrinsics=false so both
        // nodes apply the YAML extrinsic once, to raw sensor-frame clouds.
        tf_extrinsics_ = CommonUtils::readParam<bool>(nh->get_node_parameters_interface(), "tf_extrinsics", true);
        tf_wait_timeout_ = CommonUtils::readParam<double>(nh->get_node_parameters_interface(), "tf_wait_timeout", 10.0);

        pub_global_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("global_map",
            rclcpp::QoS(2).reliable());
        ds_filter_each_scan.setLeafSize(0.2, 0.2, 0.2);
        pc_last.reset(new typename pcl::PointCloud<PointType>());
        pc_last_ds.reset(new typename pcl::PointCloud<PointType>());
        pc.reset(new typename pcl::PointCloud<PointType>());
        pc_pending_.reset(new typename pcl::PointCloud<PointType>());
    }

    Eigen::Affine3d getLidarToBaselink()
    {
        return lidar_to_baselink_;
    }

    Eigen::Affine3d getImuToBaselink()
    {
        return imu_to_baselink_;
    }

    // Single choke point for buffering a downsampled scan for the worker
    // (overload hardening 2026-07-07 — replaces 7 identical inline cap blocks).
    // Drops the OLDEST scans when the worker stalls: map holes are preferred
    // to OOM, but they are now counted (cap_dropped in the 2 s summary) and
    // WARNed instead of silent. Called from sensor callbacks only (serialized
    // on cb_mtx_); the counter is atomic because processScan's summary reads
    // it from the worker thread.
    void pushScanCapped(const typename pcl::PointCloud<PointType>& cloud)
    {
        size_t n_dropped = 0;
        {
            std::lock_guard<std::mutex> lock(mtx);
            while (pc_L_buff.size() >= static_cast<size_t>(max_scan_buffer_)) {
                pc_L_buff.pop_front();
                ++n_dropped;
            }
            pc_L_buff.push_back(cloud);
        }
        if (n_dropped > 0) {
            total_cap_dropped_.fetch_add(n_dropped, std::memory_order_relaxed);
            RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), 5000,
                "[Mapping] scan buffer full (map/max_scan_buffer=%d): dropping oldest "
                "scans — the map will have holes here (%lu dropped total). The worker "
                "is not keeping up (CPU contention or spline gate).",
                max_scan_buffer_,
                static_cast<unsigned long>(total_cap_dropped_.load(std::memory_order_relaxed)));
        }
    }

    // Single definition of the sensor-callback guard policy: serialize the
    // callback body on cb_mtx_ (see above) and convert any exception into a
    // dropped scan + throttled WARN — a torn/corrupt DDS message must never
    // std::terminate the node (HARDENING Phase 6.1).
    template <typename Fn>
    void guardedCallback(Fn&& body)
    {
        std::lock_guard<std::mutex> cb_lock(cb_mtx_);
        try {
            body();
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), 5000,
                "[Mapping] sensor callback dropped a scan on exception: %s", e.what());
        }
    }

    // Returns true if any scan was consumed (published or dropped-as-old) —
    // the worker uses this to pace itself. Pacing lives in process(), NOT
    // here: a sleep inside this gate used to throttle the whole worker loop
    // (est_window ingest included) to ~20 Hz, starving the spline replica.
    bool processScan(SplineState* spl, const int64_t spl_window_st_ns)
    {
        (void)spl_window_st_ns;
        // Tally outcomes per call so the throttled summary log below
        // shows where every scan went: published, dropped-as-old, or
        // gated by spline-not-yet-caught-up.
        int n_published = 0, n_dropped_old = 0, n_pending_new = 0;
        int64_t last_t_end_ns = 0;
        // Map lag (HARDENING §6.3 mitigation): hold each scan until the spline
        // edge has advanced deskew_lag_knots_ * dt past its end, so the deskew
        // reads knots that later est_windows have already refined via
        // updateKnots/setOneStateKnot — not the under-observed trailing edge
        // whose yaw jitter smears the accumulated map under aggressive motion.
        // Costs that much map latency; 0 = publish at the bleeding edge.
        const int64_t lag_ns =
            deskew_lag_knots_ * std::max<int64_t>(spl->getKnotTimeIntervalNs(), 0);
        while (true) {
            // Pull the next cloud out of the buffer entirely under the lock
            // before doing any work. Reading pc_L_buff.front() outside the
            // lock raced the callback's buffer-cap pop_front: front() returns
            // a reference that pop_front invalidates → UAF read of the cloud
            // header. Compute t_end_ns under the lock too for the same reason.
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
                // Scan end = MAX per-point time offset. The buffered cloud is
                // VoxelGrid-filtered: points are re-ordered by voxel index and
                // fields are centroid-averaged, so back().intensity is neither
                // the last point nor a raw timestamp (finding #16). Cache per
                // scan — the lag gate below re-tests the same front for many
                // worker cycles.
                int64_t t_end_ns;
                if (front.header.stamp == t_end_cache_stamp_) {
                    t_end_ns = t_end_cache_;
                } else {
                    float max_ofs_ms = 0.f;
                    for (const auto& p : front.points) {
                        max_ofs_ms = std::max(max_ofs_ms, p.intensity);
                    }
                    t_end_ns = int64_t(front.header.stamp) +
                        int64_t(max_ofs_ms * float(1e6));
                    t_end_cache_stamp_ = front.header.stamp;
                    t_end_cache_ = t_end_ns;
                }
                last_t_end_ns = t_end_ns;
                if (t_end_ns < spl->minTimeNs()) {
                    pc_L_buff.pop_front();
                    n_dropped_old++;
                    continue;
                }
                if (t_end_ns > spl->maxTimeNs() - lag_ns) {
                    // Front spans beyond the lagged spline window — retry
                    // next cycle once more (refined) knots are available.
                    // Count SCANS that hit the gate, not worker cycles: the
                    // 5 ms cycle would otherwise inflate this ~200/s and the
                    // periodic totals would read as a failure mode.
                    if (t_end_ns != last_gated_t_end_ns_) {
                        last_gated_t_end_ns_ = t_end_ns;
                        n_pending_new++;
                    }
                    break;
                }
                // Move the cloud out of the deque (deque move of
                // pcl::PointCloud is a pointer swap) so the transform +
                // publish run without blocking the callback.
                front_cloud = std::move(pc_L_buff.front());
                pc_L_buff.pop_front();
            }
            transformCloud(front_cloud, spl, pc);
            if (publish_min_interval_ms_ <= 0) {
                publishMap(pc, pub_global_map);
            } else {
                // Batch, don't drop (overload hardening 2026-07-07): every
                // /global_map msg is ONE incremental scan that viewers
                // accumulate, so a skipped publish would be a permanent map
                // hole. Accumulate and flush on the interval below instead —
                // fewer, larger messages for the same total content.
                *pc_pending_ += *pc;
            }
            n_published++;
        }
        // Interval flush for the batch mode. Runs even on ticks that consumed
        // no scans so a gated/paused stream still drains the pending batch.
        if (publish_min_interval_ms_ > 0 && !pc_pending_->points.empty()) {
            const auto now_pub = node_handle_->get_clock()->now();
            if ((now_pub - last_batch_publish_).nanoseconds() >=
                int64_t(publish_min_interval_ms_) * 1000000LL) {
                publishMap(pc_pending_, pub_global_map);
                pc_pending_->clear();
                last_batch_publish_ = now_pub;
            }
        }
        // Update aggregate counters (cheap; called once per process-loop tick).
        total_published_ += n_published;
        total_dropped_old_ += n_dropped_old;
        total_pending_new_ += n_pending_new;
        // Throttled summary: every 2s, dump where scans are landing.
        // Reading out spline window + buffer state makes it obvious if we're
        // pinned in any of the failure modes (e.g., t_end always > spline_max).
        auto now_clk = node_handle_->get_clock()->now();
        if ((now_clk - last_processScan_log_).seconds() >= 2.0) {
            last_processScan_log_ = now_clk;
            int64_t s_min = spl->minTimeNs(), s_max = spl->maxTimeNs();
            // Read the buffer depth under the lock — the callback mutates the
            // deque concurrently, so an unlocked size() raced it (Phase 6.1).
            size_t buf_remaining;
            { std::lock_guard<std::mutex> lock(mtx); buf_remaining = pc_L_buff.size(); }
            RCLCPP_INFO(node_handle_->get_logger(),
                "[Mapping] processScan: pc_L_buff=%zu spline_knots=%ld "
                "spline=[%ld..%ld] last_t_end=%ld "
                "totals: published=%zu dropped_old=%zu pending_new=%zu cap_dropped=%lu",
                buf_remaining, spl->numKnots(), s_min, s_max, last_t_end_ns,
                total_published_, total_dropped_old_, total_pending_new_,
                static_cast<unsigned long>(total_cap_dropped_.load(std::memory_order_relaxed)));
        }
        return (n_published + n_dropped_old) > 0;
    }

    void publishMap(const typename pcl::PointCloud<PointType>::Ptr& pcs,
                         const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
    {
        if (!publishMap_logged_) {
            publishMap_logged_ = true;
            RCLCPP_INFO(node_handle_->get_logger(),
                "[Mapping] first publishMap on /global_map (points=%zu)",
                pcs->points.size());
        }
        sensor_msgs::msg::PointCloud2 msgs;
        pcl::toROSMsg(*pcs, msgs);
        msgs.header.frame_id = map_id;
        publisher->publish(msgs);
    }

    bool updateTransform(std::string source_frame_id)
    {
        if (!have_lidar_transform_) {
            // YAML-only mode (tf_extrinsics=false): identity TF; the YAML
            // q_lb/t_lb is applied once in transformPoint (findings #14/#22).
            if (!tf_extrinsics_) {
                have_lidar_transform_ = true;
                have_imu_transform_ = true;
                return true;
            }
            try {
                // Pre-check both frames exist in the buffer to avoid tf2's
                // "Invalid frame ID … - frame does not exist" warning
                // before the static TF publisher has propagated.
                if (tf_buffer_->_frameExists(this->frame_id) &&
                    tf_buffer_->_frameExists(source_frame_id) &&
                    tf_buffer_->canTransform(this->frame_id, source_frame_id,
                                             rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0))) {
                    const auto transform = tf_buffer_->lookupTransform(
                        this->frame_id, source_frame_id, rclcpp::Time(0));
                    lidar_to_baselink_ = tf2::transformToEigen(transform);
                    have_lidar_transform_ = true;
                    Eigen::Translation3d translation(lidar.t_lb);
                    Eigen::Affine3d imu_to_lidar = translation * lidar.q_lb;
                    imu_to_baselink_ = lidar_to_baselink_ * imu_to_lidar;
                    baselink_to_imu_ = imu_to_baselink_.inverse();
                    have_imu_transform_ = true;
                    RCLCPP_INFO(node_handle_->get_logger(), "[Mapping] Got LiDAR transform: %s -> %s",
                            source_frame_id.c_str(), this->frame_id.c_str());
                    return true;
                }
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), 5000,
                                    "[Mapping] LiDAR transform exception: %s", ex.what());
            }
            // Timed fallback (mirrors RESPLE's updateLidarTransform, finding
            // #22): degrade to the YAML-only convention instead of dropping
            // scans forever when no TF is ever published.
            const auto now_mono = std::chrono::steady_clock::now();
            if (!tf_first_attempt_set_) {
                tf_first_attempt_ = now_mono;
                tf_first_attempt_set_ = true;
            } else if (std::chrono::duration<double>(now_mono - tf_first_attempt_).count()
                           > tf_wait_timeout_) {
                RCLCPP_WARN(node_handle_->get_logger(),
                    "[Mapping] No TF %s -> %s after %.1f s; falling back to the YAML "
                    "q_lb/t_lb extrinsic only (identity TF).",
                    source_frame_id.c_str(), this->frame_id.c_str(), tf_wait_timeout_);
                have_lidar_transform_ = true;
                have_imu_transform_ = true;
                return true;
            }
            RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), 5000,
                                "[Mapping] Waiting for LiDAR transform: %s -> %s",
                                source_frame_id.c_str(), this->frame_id.c_str());
            return false;
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
        // Mirror RESPLE's Association::pointBodyToWorld EXACTLY: the spline is
        // the world<-base_link trajectory (RESPLE applies it to TF-transformed
        // base_link points, with the YAML q_bl/t_bl as an extra offset that is
        // identity in TF mode). The previous base_link -> "IMU" hop through
        // baselink_to_imu_ = (TF * (t_lb,q_lb))^-1 re-applied the inverse TF
        // extrinsic, rotating the whole map by the mounting transform relative
        // to RESPLE's odometry whenever the TF is non-identity (bug-hunt
        // 2026-07-02 finding #14). In YAML-only mode (tf_extrinsics=false) the
        // buffered points are raw sensor-frame and q_bl/t_bl applies once,
        // exactly like RESPLE.
        Eigen::Vector3d p_body(pt_in.x, pt_in.y, pt_in.z);
        Eigen::Vector3d p_global(q_itp * (lidar.q_bl * p_body + lidar.t_bl) + t_itp);
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
        pc_out->clear();
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
    // Diagnostic counters + last-log time (see processScan / publishMap).
    bool publishMap_logged_ = false;
    rclcpp::Time last_processScan_log_{0, 0, RCL_ROS_TIME};
    size_t total_published_ = 0;
    size_t total_dropped_old_ = 0;
    size_t total_pending_new_ = 0;
    // Scans evicted by the buffer cap (pushScanCapped). Atomic: written on
    // the callback threads, read by the worker's 2 s summary.
    std::atomic<uint64_t> total_cap_dropped_{0};
    int max_scan_buffer_ = 200;
    // Batch publish (map/publish_min_interval_ms, 0 = publish every scan).
    // Worker-thread only.
    int publish_min_interval_ms_ = 0;
    rclcpp::Time last_batch_publish_{0, 0, RCL_ROS_TIME};
    typename pcl::PointCloud<PointType>::Ptr pc_pending_;
    // Last gate-held front t_end — dedupes pending_new across worker cycles.
    int64_t last_gated_t_end_ns_ = 0;
    typename pcl::PointCloud<PointType>::Ptr pc_last;
    typename pcl::PointCloud<PointType>::Ptr pc_last_ds;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_each_scan;
    std::string frame_id = "base_link";
    std::string map_id = "map";
    int num_threads_ = 5;
    // Scans wait until the spline edge is this many knots past their end
    // before being deskewed into the map (map_deskew_lag_knots, HARDENING §6.3).
    int deskew_lag_knots_ = 8;
    
    // TF transformation
    rclcpp::Node::SharedPtr node_handle_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_lidar_transform_;
    bool have_imu_transform_;
    Eigen::Affine3d lidar_to_baselink_;
    Eigen::Affine3d imu_to_baselink_;
    Eigen::Affine3d baselink_to_imu_;
    // Extrinsic convention switch — mirrors RESPLE's (findings #14/#22):
    // false = dataset/upstream mode, no TF consulted, YAML q_lb/t_lb only.
    bool tf_extrinsics_ = true;
    double tf_wait_timeout_ = 10.0;
    std::chrono::steady_clock::time_point tf_first_attempt_{};
    bool tf_first_attempt_set_ = false;
    // Per-scan cache for the max-point-time scan end (finding #16).
    std::uint64_t t_end_cache_stamp_ = 0;
    int64_t t_end_cache_ = 0;

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
        this->guardedCallback([&] {
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
        this->pushScanCapped(*pc_last_ds);
        });
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_ouster;
    int64_t time_offset = 0;
};

// Generic PointCloud2 buffer: mirrors OusterBuff but uses runtime PointField
// introspection (utils/point_cloud_adapter.h), so the map side accepts the
// same arbitrary PointCloud2 layouts as RESPLE's genericLidarCallback.
class GenericPC2Buff : public MappingBase<pcl::PointXYZINormal>
{
  public:
  GenericPC2Buff(rclcpp::Node::SharedPtr &nh, const LidarConfig& lidar_config,
                 rclcpp::CallbackGroup::SharedPtr sensor_cb = nullptr)
      : MappingBase<pcl::PointXYZINormal>(nh, lidar_config, sensor_cb)
    {
        adapter_cfg_ = resple::makeAdapterConfig(
            lidar_config.time_field, lidar_config.time_unit, lidar_config.intensity_field);
        pc_subscription_generic = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->lidar.topic, lidar_qos, std::bind(&GenericPC2Buff::genericLidarCallback, this, std::placeholders::_1), sub_opt);
        double lidar_time_offset = CommonUtils::readParam<double>(nh->get_node_parameters_interface(), "lidar_time_offset", 0.0);
        time_offset = 1e9*lidar_time_offset;
    }

    void genericLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
    {
        this->guardedCallback([&] {
        // Guard against negative timestamps (sim-time messages)
        int64_t stamp_ns = rclcpp::Time(msg_in->header.stamp).nanoseconds();
        if (stamp_ns < time_offset) return;

        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(msg_in->header.frame_id)) return;

        // Field-introspecting conversion. The map keeps the full cloud:
        // no blind-radius cut and no decimation (the voxel filter below
        // bounds the density), matching the per-sensor map buffers.
        this->pc_last->clear();
        if (!resple::ingestPointCloud2(*msg_in, /*blind=*/0.f,
                                       /*point_filter_num=*/1, adapter_cfg_,
                                       *this->pc_last)) {
            RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *node_handle_->get_clock(), 5000,
                "[Mapping] PointCloud2 on '%s' has no usable x/y/z layout — dropped",
                this->lidar.topic.c_str());
            return;
        }
        if (this->pc_last->points.empty()) return;
        this->pc_last->header.frame_id = this->frame_id;
        this->pc_last->header.stamp = stamp_ns - time_offset;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*this->pc_last, *this->pc_last, indices);
        if (this->pc_last->points.empty()) return;

        // Transform point cloud to body frame
        pcl::transformPointCloud(*this->pc_last, *this->pc_last, lidar_to_baselink_);

        ds_filter_each_scan.setInputCloud(pc_last);
        this->pc_last_ds->clear();
        ds_filter_each_scan.filter(*this->pc_last_ds);
        pc_last_ds->header.frame_id = this->frame_id;
        pc_last_ds->header.stamp = stamp_ns - time_offset;
        this->pushScanCapped(*pc_last_ds);
        });
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_generic;
    resple::pc2::AdapterConfig adapter_cfg_;
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
        this->guardedCallback([&] {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(livox_msg_in->header.frame_id)) return;
        
        this->pc_last->clear();
        // DDS robustness: a torn/corrupt message can ship point_num disagreeing
        // with the actual points vector — clamp to the vector to avoid OOB reads.
        const size_t plsize = std::min<size_t>(livox_msg_in->point_num, livox_msg_in->points.size());
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        // Start at 0: RESPLE.cpp's livox ingest starts at 1 because points[0]
        // seeds pt_pre for its duplicate-point filter; this loop has no pt_pre,
        // so starting at 1 just dropped one valid point per scan.
        for (size_t i = 0; i < plsize; i++) {
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
        this->pushScanCapped(*pc_last_ds);
        });
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
        this->guardedCallback([&] {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(livox_msg_in->header.frame_id)) return;
        
        this->pc_last->clear();
        // DDS robustness: a torn/corrupt message can ship point_num disagreeing
        // with the actual points vector — clamp to the vector to avoid OOB reads.
        const size_t plsize = std::min<size_t>(livox_msg_in->point_num, livox_msg_in->points.size());
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        // Start at 0: RESPLE.cpp's livox ingest starts at 1 because points[0]
        // seeds pt_pre for its duplicate-point filter; this loop has no pt_pre,
        // so starting at 1 just dropped one valid point per scan.
        for (size_t i = 0; i < plsize; i++) {
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
        this->pushScanCapped(*pc_last_ds);
        });
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
        this->guardedCallback([&] {
        // Lookup LiDAR transform if not yet initialized
        if(!updateTransform(livox_msg_in->header.frame_id)) return;
                
        this->pc_last->clear();
        // DDS robustness: a torn/corrupt message can ship point_num disagreeing
        // with the actual points vector — clamp to the vector to avoid OOB reads.
        const size_t plsize = std::min<size_t>(livox_msg_in->point_num, livox_msg_in->points.size());
        if (plsize == 0) return;
        this->pc_last->reserve(plsize);
        pcl::PointXYZINormal pt;
        // Start at 0: RESPLE.cpp's livox ingest starts at 1 because points[0]
        // seeds pt_pre for its duplicate-point filter; this loop has no pt_pre,
        // so starting at 1 just dropped one valid point per scan.
        for (size_t i = 0; i < plsize; i++) {
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
        this->pushScanCapped(*pc_last_ds);
        });
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
        this->guardedCallback([&] {
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
        this->pushScanCapped(*pc_last_ds);
        });
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
        this->guardedCallback([&] {
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
        this->pushScanCapped(*pc_last_ds);
        });
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

        // Initialize TF buffer and listener.
        // Bump cache_time to 30s (default 10s) so a brief stall in
        // RESPLE's pose stream doesn't immediately starve our lookups.
        tf_buffer = std::make_shared<tf2_ros::Buffer>(
            this->get_clock(),
            tf2::durationFromSec(30.0));
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Read frame ID parameters
        frame_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "frame_id", "base_link");
        odom_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "odom/frame_id", "odom");
        map_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "map/frame_id", "map");

        publish_tf = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "map/publish_tf", true);
        invert_tf  = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "map/invert_tf", false);

        // TF ownership guard (2026-07-07): same scheme as the RESPLE node
        // (see utils/tf_ownership.h), watching the map->odom pair this node
        // broadcasts. Shared param names — one knob configures both nodes.
        tf_conflict_action_ = CommonUtils::readParam<std::string>(
            this->get_node_parameters_interface(), "tf_conflict_action", std::string("warn"));
        if (tf_conflict_action_ != "warn" && tf_conflict_action_ != "yield") {
            RCLCPP_WARN(this->get_logger(),
                "[Mapping] tf_conflict_action='%s' unknown (warn|yield); using 'warn'",
                tf_conflict_action_.c_str());
            tf_conflict_action_ = "warn";
        }
        tf_conflict_quiet_ns_ = static_cast<int64_t>(1e9 * CommonUtils::readParam<double>(
            this->get_node_parameters_interface(), "tf_conflict_quiet_sec", 5.0));
        tf_absent_warn_sec_ = CommonUtils::readParam<double>(
            this->get_node_parameters_interface(), "tf_absent_warn_sec", 10.0);
        if (invert_tf) {
            tf_own_monitor_.configure(odom_id, map_id);
        } else {
            tf_own_monitor_.configure(map_id, odom_id);
        }
        tf_absence_warned_.store(false, std::memory_order_relaxed);
        tf_yield_active_.store(false, std::memory_order_relaxed);
        // amcl-style future-dating for the map->odom TF (seconds added to the
        // stamp). The transform is stamped at the (lagged) path tip, so exact-
        // time lookups in the map frame at fresh sensor stamps fail with
        // "extrapolation into the future". Future-dating the slowly-varying
        // drift correction is the standard remedy; 0 = off (stamp at the tip).
        map_tf_tolerance_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "map/transform_tolerance", 0.0);
        if (map_tf_tolerance_ < 0.0) map_tf_tolerance_ = 0.0;

        // HARDENING §3.1: same retention parameter as the RESPLE node.
        // spline_active_ otherwise grows one knot per est_window knot for the
        // whole run. Retention must stay wider than the publish/scan lag
        // (publishPath walks path_t_ns_ forward each cycle; processScan only
        // consumes scans inside the latest window), so clamp like RESPLE does.
        spline_prune_keep_knots_ = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "spline_prune_keep_knots", 600);
        if (spline_prune_keep_knots_ != 0 && spline_prune_keep_knots_ < 100) {
            RCLCPP_WARN(this->get_logger(),
                "spline_prune_keep_knots=%d is below the safe minimum; using 100",
                spline_prune_keep_knots_);
            spline_prune_keep_knots_ = 100;
        }
        // Same lag the per-sensor buffers use for the map (MappingBase reads
        // it too; readParam's has_parameter guard makes the double read safe).
        // Here it lags the path tip, so the map->odom TF composed in pubOdom
        // is built from fully-converged knots (see publishPath).
        map_deskew_lag_knots_ = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "map_deskew_lag_knots", 8);
        if (map_deskew_lag_knots_ < 0) map_deskew_lag_knots_ = 0;

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
        static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

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
        
        // Create subscriptions.
        // start_time uses transient_local to match RESPLE's publisher
        // QoS, so this subscription still receives the one-shot signal
        // even if it activates after RESPLE has already published.
        auto start_qos = rclcpp::QoS(1).transient_local().reliable();
        auto large_reliable_qos = rclcpp::QoS(10000).reliable();
        sub_start = this->create_subscription<std_msgs::msg::Int64>("start_time", start_qos,
            std::bind(&Mapping::startCallBack, this, std::placeholders::_1));
        sub_est = this->create_subscription<estimate_msgs::msg::Estimate>("est_window", large_reliable_qos,
            std::bind(&Mapping::getEstCallback, this, std::placeholders::_1));
        // TF ownership guard: raw /tf + /tf_static watchers (mirror of the
        // RESPLE node's; see tfWatchCallback there for the rationale).
        sub_tf_watch_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::QoS(rclcpp::KeepLast(100)),
            std::bind(&Mapping::tfWatchCallback, this, std::placeholders::_1));
        sub_tf_static_watch_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", rclcpp::QoS(rclcpp::KeepLast(100)).transient_local(),
            std::bind(&Mapping::tfWatchCallback, this, std::placeholders::_1));
        tf_watch_start_ = std::chrono::steady_clock::now();
        
        // Start processing thread (exited-flag store last, for the bounded
        // join's join-vs-detach decision — see joinProcessingThreadBounded).
        processing_active_ = true;
        processing_thread_exited_.store(false, std::memory_order_release);
        processing_thread_ = std::thread([this] {
            process();
            processing_thread_exited_.store(true, std::memory_order_release);
        });
        
        RCLCPP_INFO(this->get_logger(), "Mapping activated successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating Mapping...");

        processing_active_ = false;
        joinProcessingThreadBounded(std::chrono::seconds(2));
        
        // Deactivate publishers
        pub_path->on_deactivate();
        pub_knots->on_deactivate();
        pub_odom->on_deactivate();
        
        // Reset subscriptions
        sub_start.reset();
        sub_est.reset();
        sub_tf_watch_.reset();
        sub_tf_static_watch_.reset();

        // Clear init flag so a re-activation starts in the unitialized state.
        // Worker is joined; safe to write without ordering concerns. Also drop
        // any restart staged but not yet consumed by the (now-joined) worker, so
        // it can't fire with a stale bag time after the next activation.
        if_init_succeed.store(false, std::memory_order_relaxed);
        start_pending_.store(false, std::memory_order_relaxed);

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
        static_br_.reset();

        // Clear data
        opt_old_path.poses.clear();
        path_t_ns_ = 0;
        if_init_succeed = false;
        // Allow re-activation to re-latch the identity map→odom TF.
        static_map_odom_sent_.store(false);
        
        RCLCPP_INFO(this->get_logger(), "Mapping cleaned up successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Mapping...");

        processing_active_ = false;
        // Bounded join: under SIGINT we have ~5s before SIGTERM. If
        // process() is wedged inside processScan / lookupTransform, detach
        // rather than hang the launcher.
        joinProcessingThreadBounded(std::chrono::seconds(2));
        
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

    // Bounded join. If the worker is wedged (TF lookup, third-party lock),
    // detach so shutdown can complete inside the launcher's 5s SIGINT→SIGTERM
    // window. Poll-based for the same reason as RESPLE's twin (see the
    // comment there): the previous std::async-join + timeout-detach raced two
    // threads on the same std::thread object (UB when the worker exited right
    // at the deadline), and the future destructor un-bounded the wait.
    void joinProcessingThreadBounded(std::chrono::milliseconds timeout)
    {
        if (!processing_thread_.joinable()) return;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!processing_thread_exited_.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (processing_thread_exited_.load(std::memory_order_acquire)) {
            processing_thread_.join();
        } else {
            processing_thread_.detach();
            RCLCPP_WARN(this->get_logger(),
                "Mapping process thread did not exit within timeout; detached");
        }
    }

    void process() {
        // RCLCPP_INFO(this->get_logger(), "process");
        // See processScan for the rationale on std::this_thread::sleep_for
        // vs rclcpp::Rate: rate.sleep() throws on SIGINT-invalidated context.
        constexpr auto kRatePeriod = std::chrono::milliseconds(100);  // 10 Hz
        int64_t num_knot = 0;
        while (processing_active_ && rclcpp::ok()) {
            // Option B: consume a (re)start staged by startCallBack and perform
            // the spline_active_.init() here, on the worker thread, under
            // m_spline. Doing it before the swap below means a re-init clears the
            // window first; we also drop any knots the previous run had staged so
            // they aren't swapped onto the fresh spline. Because this is the only
            // place (besides the swap) that mutates spline_active_, the publish
            // reads further down are genuinely single-threaded w.r.t. it.
            if (start_pending_.exchange(false, std::memory_order_acquire)) {
                {
                    std::lock_guard<std::mutex> lock(m_spline);
                    // dt=1 placeholder; overridden by getEstCallback's first window.
                    spline_active_.init(1, 0, start_bag_time_pending_.load(std::memory_order_relaxed), 0);
                    // Windows staged by the previous run target its indexing.
                    est_window_queue_.clear();
                    est_windows_dropped_ = 0;
                    spline_pending_ready_.store(false, std::memory_order_relaxed);
                }
                // Only now is spline_active_ init'd: publish that to consumers.
                // startCallBack set if_init_succeed=false, so until this store no
                // reader can act on a default-constructed / stale spline.
                if_init_succeed.store(true, std::memory_order_release);
            }
            // Drain ALL staged est_windows in order (lock-free check then
            // mutex drain). Applying every window keeps the replica's knot
            // indexing exactly in step with the estimator — see the
            // est_window_queue_ member comment for the drift this prevents.
            if (spline_pending_ready_.load()) {
                std::lock_guard<std::mutex> lock(m_spline);
                if (spline_pending_ready_.load()) {
                    ScopedMappingsLock maps_lock(vis_maps);
                    while (!est_window_queue_.empty()) {
                        EstWindow& w = est_window_queue_.front();
                        spl_window_st_ns = w.window_st_ns;
                        spline_active_.setTimeIntervalNs(w.spline.getKnotTimeIntervalNs());
                        spline_active_.updateKnots(&w.spline);
                        est_window_queue_.pop_front();
                    }
                    spline_pending_ready_.store(false);
                }
            }
            // Consume a (re)start signal: reset worker-local trackers so a
            // RESPLE respawn doesn't leave us gating on a stale knot count /
            // path timestamp from the previous run. Done on the worker thread
            // so opt_old_path/path_t_ns_/num_knot are only ever touched here.
            if (path_reset_.exchange(false, std::memory_order_acquire)) {
                num_knot = 0;
                path_t_ns_ = 0;
                opt_old_path.poses.clear();
            }
            // TF ownership absence check (one-shot, mirror of the RESPLE
            // node's): map/publish_tf=false means an external localizer is
            // expected to own map->odom — WARN if nobody has published it.
            if (!publish_tf && tf_absent_warn_sec_ > 0 &&
                !tf_absence_warned_.load(std::memory_order_relaxed) &&
                !tf_own_monitor_.pairSeen()) {
                const double waited = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - tf_watch_start_).count();
                if (waited > tf_absent_warn_sec_) {
                    tf_absence_warned_.store(true, std::memory_order_relaxed);
                    RCLCPP_WARN(this->get_logger(),
                        "[Mapping] map/publish_tf=false but NO ONE has published "
                        "%s->%s in %.0f s. The external owner (global localizer?) "
                        "is not wired up - the map frame is dangling. Either start "
                        "it or set map/publish_tf: true.",
                        tf_own_monitor_.parent().c_str(),
                        tf_own_monitor_.child().c_str(), waited);
                }
            }
            // Acquire-load on if_init_succeed pairs with startCallBack's
            // release-store so the staged start_pending_ / start_bag_time_pending_
            // are visible. Under Option B all spline_active_ mutation (init above
            // + swap above) is on this worker thread, so these reads are now
            // genuinely single-threaded with respect to spline_active_ — no
            // m_spline coverage needed here.
            // Throttle the publish block to the old ~20 Hz cadence: knots now
            // land at est rate (~100 Hz) since the queue drain, and each
            // publishPath ships the entire accumulated Path message.
            const auto now_steady = std::chrono::steady_clock::now();
            if (if_init_succeed.load(std::memory_order_acquire) && spline_active_.totalKnots() > num_knot
                && now_steady - last_pub_block_ >= std::chrono::milliseconds(50)) {
                last_pub_block_ = now_steady;
                // Lock order MUST match the swap branch above (m_spline -> maps).
                // This branch previously took maps (ScopedMappingsLock) first and
                // m_spline only for the prune, giving maps -> m_spline — a
                // lock-order inversion / potential deadlock with the swap branch
                // (TSan, Phase 6.1). Acquire m_spline first, then the maps.
                std::lock_guard<std::mutex> spline_lock(m_spline);
                ScopedMappingsLock maps_lock(vis_maps);
                publishPath();
                displayControlPoints();
                pubOdom();
                // totalKnots() (monotonic, includes pruned) rather than
                // numKnots(): once Phase 3.1 pruning caps the retained count
                // at the retention window, a numKnots() gate would never fire
                // again and path/odom would silently freeze.
                num_knot = spline_active_.totalKnots();
                // Phase 3.1: prune after the publishes consumed this growth
                // step. m_spline is already held above (serializes against the
                // swap branch); all spline_active_ mutation is on this worker
                // thread (Option B), so no additional locking is needed.
                if (spline_prune_keep_knots_ > 0) {
                    spline_active_.pruneFrontKnots(spline_prune_keep_knots_);
                }
            }
            if (!if_init_succeed.load(std::memory_order_acquire)) {
                // chrono sleep so SIGINT-invalidated context never throws here.
                std::this_thread::sleep_for(kRatePeriod);
                continue;
            }
            bool progress = false;
            for (const auto vis_map : vis_maps) {
                progress = vis_map->processScan(&spline_active_, spl_window_st_ns) || progress;
            }
            // Single pacing point (sleeps used to live inside processScan's
            // gate branch, which throttled the whole worker — including the
            // est_window drain above — to ~20 Hz). 5 ms: gate retries and
            // queue drains stay prompt, no busy-spin when idle.
            if (!progress && !spline_pending_ready_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

private:
    std::string node_name = "Mapping";
    int64_t spl_window_st_ns;
    // Spline replica: getEstCallback enqueues est_windows, process() applies
    // them to spline_active_ under m_spline — the worker remains the only
    // thread that mutates spline_active_ (no callback-vs-worker race).
    SplineState spline_active_;
    // Lossless est_window ingest queue (guarded by m_spline). The previous
    // single-slot double buffer (spline_pending_) kept only the LATEST
    // window: every window arriving while the worker was mid-cycle was
    // overwritten, and updateKnots then appended across the index gap —
    // compressing the replica's time axis. Symptoms: traj_path//odometry
    // stamps drifting ~0.7-10% of elapsed time vs their poses, and the
    // deskew consumer pacing itself to the slowed replica edge (the
    // apparent Avia-frame "saturation"). The worker drains the whole queue
    // in order each cycle, so the replica applies every window exactly as
    // the estimator emitted it.
    struct EstWindow { SplineState spline; int64_t window_st_ns; };
    Eigen::aligned_deque<EstWindow> est_window_queue_;
    // ~20 s of windows @ 100 Hz est rate. Overflow means the worker stalled
    // that long; drop oldest with a warn (the updateKnots gap invariant then
    // marks the misalignment, which the next RESPLE restart clears).
    static constexpr size_t kEstWindowQueueCap = 2000;
    size_t est_windows_dropped_ = 0;                 // guarded by m_spline
    std::atomic<bool> spline_pending_ready_{false};  // queue non-empty hint
    // Pace for the path/odom/control-point publishes: knots now arrive at
    // est rate (~100 Hz) instead of the old ~20 Hz swap rate, and each
    // publishPath ships the whole accumulated Path msg — re-publishing that
    // per knot would be pure bandwidth. 50 ms keeps the old ~20 Hz cadence.
    std::chrono::steady_clock::time_point last_pub_block_{};
    rclcpp::Subscription<estimate_msgs::msg::Estimate>::SharedPtr sub_est;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_start;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    nav_msgs::msg::Path opt_old_path;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_knots;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    bool getEstCallback_logged_ = false;
    // Lifecycle management
    std::atomic<bool> processing_active_;
    std::thread processing_thread_;
    // Set by the worker lambda as its final action; consumed by
    // joinProcessingThreadBounded (join-vs-detach without concurrent access
    // to processing_thread_).
    std::atomic<bool> processing_thread_exited_{false};
    std::vector<MappingBase<pcl::PointXYZINormal>*> vis_maps;
    std::string frame_id;
    std::string odom_id;
    std::string map_id;
    Eigen::Vector<double, 6> cov_pose;
    Eigen::Vector<double, 6> cov_twist;
    bool publish_tf, invert_tf;
    // map->odom TF future-dating in seconds (map/transform_tolerance, 0 = off).
    double map_tf_tolerance_ = 0.0;
    // TF ownership guard (2026-07-07): mirror of the RESPLE node's, watching
    // the (post-inversion) map->odom pair this node broadcasts. Both the
    // dynamic transform and the static identity latch are whitelisted by
    // stamp (notePublished / notePublishedStatic). Shared param names with
    // RESPLE (tf_conflict_action / tf_conflict_quiet_sec / tf_absent_warn_sec)
    // — both nodes read the same YAML.
    resple::tfown::TfOwnershipMonitor tf_own_monitor_;
    std::string tf_conflict_action_ = "warn";
    int64_t tf_conflict_quiet_ns_ = 5000000000LL;
    double tf_absent_warn_sec_ = 10.0;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_watch_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_static_watch_;
    std::atomic<bool> tf_absence_warned_{false};
    std::atomic<bool> tf_yield_active_{false};
    std::chrono::steady_clock::time_point tf_watch_start_{};

    static int64_t steadyNowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void tfWatchCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        const int64_t now_ns = steadyNowNs();
        for (const auto& ts : msg->transforms) {
            const int64_t stamp_ns = rclcpp::Time(ts.header.stamp).nanoseconds();
            const auto v = tf_own_monitor_.classify(
                ts.header.frame_id, ts.child_frame_id, stamp_ns, now_ns);
            if (!publish_tf) continue;  // demoted: foreign owner is EXPECTED
            if (v == resple::tfown::Verdict::FOREIGN_SAME_PAIR) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[Mapping] TF ownership conflict: another publisher is broadcasting "
                    "%s->%s (%lu foreign transforms so far). Two publishers on one TF "
                    "pair make the map frame flicker. Either set map/publish_tf: false "
                    "here (a global localizer owns the map frame) or disable the other "
                    "publisher.%s",
                    tf_own_monitor_.parent().c_str(), tf_own_monitor_.child().c_str(),
                    static_cast<unsigned long>(tf_own_monitor_.foreignSamePair()),
                    tf_conflict_action_ == "yield"
                        ? " tf_conflict_action=yield: suspending our broadcast while "
                          "the other publisher is active." : "");
            } else if (v == resple::tfown::Verdict::FOREIGN_OTHER_PARENT) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[Mapping] TF tree conflict: frame '%s' is also claimed by parent "
                    "'%s' while we publish %s->%s (%lu so far). A TF child has exactly "
                    "one parent - fix the other publisher or our frame configuration.",
                    tf_own_monitor_.child().c_str(), ts.header.frame_id.c_str(),
                    tf_own_monitor_.parent().c_str(), tf_own_monitor_.child().c_str(),
                    static_cast<unsigned long>(tf_own_monitor_.foreignOtherParent()));
            }
        }
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    // cuda-perf-nano addition: latch identity map→odom on first
    // startCallBack so the REP-105 chain is closed during the cold-start
    // window before pubOdom()'s first odom→base_link lookup succeeds.
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
    std::atomic<bool> static_map_odom_sent_{false};
    // Set true ONLY by the worker (process()), immediately after it runs
    // spline_active_.init() on consuming start_pending_. startCallBack sets it
    // FALSE to pause consumers across a (re)start. So any thread observing
    // if_init_succeed==true is guaranteed an init'd spline_active_ — this is
    // what makes Option B's deferred init() safe (no default-constructed read).
    std::atomic<bool> if_init_succeed{false};
    int64_t path_t_ns_ = 0;
    // Set by startCallBack (executor thread) on every (re)start, consumed once
    // by process() (worker thread). On a RESPLE respawn the spline window is
    // re-init'd from a fresh start time; without this the worker's local
    // num_knot and the member path_t_ns_/opt_old_path hold stale values from
    // the previous run, so the publish gate (numKnots() > num_knot) and
    // publishPath's while-loop never advance and path/odom silently freeze.
    std::atomic<bool> path_reset_{false};
    // Option B (Mapping respawn race fix): startCallBack stages the (re)start
    // here instead of calling spline_active_.init() itself. The worker performs
    // init() under m_spline when it consumes start_pending_, so spline_active_
    // is NEVER mutated from the executor thread. This removes the data race
    // between startCallBack's init() and process()'s publish reads, which hold
    // ScopedMappingsLock (the per-map mutexes) rather than m_spline.
    std::atomic<bool> start_pending_{false};
    std::atomic<int64_t> start_bag_time_pending_{0};
    std::mutex m_spline;
    // Serializes getEstCallback. sub_est is on the node's default
    // (MutuallyExclusive) group, but under the MultiThreadedExecutor TSan sees
    // no happens-before between consecutive est_window deliveries, so the
    // unguarded getEstCallback_logged_ flag (and the spline_pending_ staging)
    // raced callback-vs-callback (HARDENING Phase 6.1, est-window data path).
    std::mutex est_cb_mtx_;
    // Phase 3.1: knots retained by the sliding-window prune (0 disables).
    int spline_prune_keep_knots_ = 600;
    // §6.3 map lag, node-level copy: lags the publishPath tip (and therefore
    // the map->odom TF composed from it in pubOdom).
    int map_deskew_lag_knots_ = 8;

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
        std::lock_guard<std::mutex> est_lock(est_cb_mtx_);  // Phase 6.1: serialize est-window handler
        try {  // Phase 6.1 DDS robustness: drop+log a malformed est_window, never terminate
        if (!getEstCallback_logged_) {
            getEstCallback_logged_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[Mapping] first /est_window received (knots=%zu, init_succeed=%d)",
                est_msg->spline.knots.size(), int(if_init_succeed.load()));
        }
        if (!if_init_succeed) {
            return;
        }

        estimate_msgs::msg::Spline spline_msg = est_msg->spline;
        // DDS robustness: a corrupt est_window with non-positive knot dt would
        // produce a degenerate spline (div-by-zero in time interpolation). Drop.
        if (spline_msg.dt <= 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[Mapping] getEstCallback dropped est_window with non-positive dt=%ld", (long)spline_msg.dt);
            return;
        }
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
        // Enqueue — process() drains the queue in order under m_spline.
        // Every window must be queued (not overwritten): each one carries its
        // own start_idx, and a skipped window leaves an index gap that
        // updateKnots can only fill by misaligning every later knot's time.
        {
            std::lock_guard<std::mutex> lock(m_spline);
            if (est_window_queue_.size() >= kEstWindowQueueCap) {
                est_window_queue_.pop_front();
                if ((++est_windows_dropped_ % 100) == 1) {
                    RCLCPP_WARN(this->get_logger(),
                        "[Mapping] est_window queue overflow (worker stalled?): "
                        "%zu windows dropped — replica knot times misaligned until restart",
                        est_windows_dropped_);
                }
            }
            est_window_queue_.push_back(
                EstWindow{std::move(spline_w), spline_msg.start_t - spline_msg.dt});
            spline_pending_ready_.store(true);
        }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[Mapping] getEstCallback dropped a malformed est_window: %s", e.what());
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
            // TF ownership guard 'yield': suspend our map->odom broadcast
            // while a foreign publisher on the pair was seen within the quiet
            // window (mirror of the RESPLE node's odom TF yield).
            const bool yield_now = tf_conflict_action_ == "yield"
                && tf_own_monitor_.foreignActiveWithin(steadyNowNs(), tf_conflict_quiet_ns_);
            tf_yield_active_.store(yield_now, std::memory_order_relaxed);
            if (yield_now) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[Mapping] yielding the %s->%s TF to a foreign publisher "
                    "(tf_conflict_action=yield); odometry/path topics continue.",
                    tf_own_monitor_.parent().c_str(), tf_own_monitor_.child().c_str());
                return;
            }
            // Calculate desired frame to map transform
            geometry_msgs::msg::TransformStamped baselink_to_map;
            baselink_to_map.header.stamp = odom_msg.header.stamp;
            baselink_to_map.header.frame_id = map_id;
            baselink_to_map.child_frame_id = frame_id;
            baselink_to_map.transform.translation.x = odom_pose_current.pose.position.x;
            baselink_to_map.transform.translation.y = odom_pose_current.pose.position.y;
            baselink_to_map.transform.translation.z = odom_pose_current.pose.position.z;
            baselink_to_map.transform.rotation = odom_pose_current.pose.orientation;

            // Get frame to odom transform AT THE PATH-TIP STAMP, so both
            // factors of the map→odom composition below are sampled at the
            // same instant. The old Time(0) ("latest") lookup paired a
            // base→map pose that is 50–180 ms old (lagged path tip + 100 ms
            // path stepping) with the freshest odom→base — that mismatch puts
            // full body motion over the gap into the map→odom TF, which is
            // exactly the yaw jitter §6.3 documents. The tip stamp is well in
            // the past by construction (the deskew/tip lag), so the exact-time
            // lookup interpolates inside buffered TF history instead of racing
            // the live edge — the failure mode that originally motivated
            // Time(0). Keep Time(0) as a fallback for warm-up, where history
            // may not reach back to the tip yet.
            geometry_msgs::msg::TransformStamped odom_to_baselink;
            bool got_odom_transform = false;
            const tf2::TimePoint tip_time = tf2::TimePoint(
                std::chrono::nanoseconds(rclcpp::Time(odom_pose_current.header.stamp).nanoseconds()));
            try {
                // No wait on the tip-stamp lookup: the stamp is in the past, so
                // either buffered history already covers it (steady state,
                // instant success) or it never will until more history accrues
                // (warm-up) — waiting can't help, and it would stack with the
                // fallback's 0.1 s wait below.
                if (tf_buffer->canTransform(this->frame_id, this->odom_id,
                                            tip_time,
                                            tf2::durationFromSec(0.0))) {
                    odom_to_baselink = tf_buffer->lookupTransform(
                        this->frame_id, this->odom_id, tip_time);
                    got_odom_transform = true;
                } else if (tf_buffer->canTransform(this->frame_id, this->odom_id,
                                                   tf2::TimePointZero,
                                                   tf2::durationFromSec(0.1))) {
                    odom_to_baselink = tf_buffer->lookupTransform(
                        this->frame_id, this->odom_id, tf2::TimePointZero);
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
                // Optional future-dating (map/transform_tolerance) so exact-time
                // lookups at stamps newer than the lagged tip still resolve.
                odom_to_map.header.stamp = rclcpp::Time(odom_msg.header.stamp) +
                    rclcpp::Duration::from_seconds(map_tf_tolerance_);
                if(invert_tf){
                    odom_to_map.header.frame_id = odom_id;
                    odom_to_map.child_frame_id  = map_id;
                } else {
                    odom_to_map.header.frame_id = map_id;
                    odom_to_map.child_frame_id  = odom_id;
                }
                // Record the exact wire stamp BEFORE sending so the DDS
                // loopback of our own message can never race the ring insert.
                tf_own_monitor_.notePublished(
                    rclcpp::Time(odom_to_map.header.stamp).nanoseconds());
                br->sendTransform(odom_to_map);
            }
        }
    }

    void startCallBack(const std_msgs::msg::Int64::SharedPtr start_time_msg)
    {
        // Option B: stage the (re)start for the worker thread rather than
        // calling spline_active_.init() here. This keeps ALL spline_active_
        // mutation on the worker, eliminating the data race with process()'s
        // publish reads (which hold ScopedMappingsLock, not m_spline). The
        // worker performs init() under m_spline when it consumes start_pending_.
        // The release-stores below pair with the worker's acquire-loads so the
        // staged bag time is visible before the worker acts on the flags.
        int64_t bag_start_time = start_time_msg->data;
        start_bag_time_pending_.store(bag_start_time, std::memory_order_relaxed);
        // Pause consumers until the worker has actually re-init'd spline_active_;
        // the worker flips if_init_succeed back to true right after init(). This
        // closes the cold-start window where a reader could observe
        // if_init_succeed==true before the deferred init() ran.
        if_init_succeed.store(false, std::memory_order_release);
        // Tell the worker to drop the previous run's path/knot trackers before
        // it publishes against the freshly re-init'd spline window.
        path_reset_.store(true, std::memory_order_release);
        // Stage the (re)start LAST: the worker's acquire-exchange on
        // start_pending_ then sees the bag time + both flags above.
        start_pending_.store(true, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(),
            "[Mapping] startCallBack: staged restart (bag_start_time=%ld)",
            bag_start_time);

        // Latch an identity map→odom so the REP-105 chain is never broken
        // during the cold-start window before pubOdom() gets its first
        // valid odom→base_link lookup. tf_static is transient_local, so
        // late-joining listeners still receive it.
        if (publish_tf && static_br_) {
            // TF ownership guard 'yield': a foreign map->odom owner active at
            // start means our identity latch would inject a conflicting
            // transform onto /tf_static (transient_local — it would linger).
            // Skip WITHOUT consuming the one-shot, so a later, quiet (re)start
            // can still latch.
            if (tf_conflict_action_ == "yield" &&
                tf_own_monitor_.foreignActiveWithin(steadyNowNs(), tf_conflict_quiet_ns_)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[Mapping] skipping the identity %s->%s static latch: a foreign "
                    "publisher owns the pair (tf_conflict_action=yield).",
                    tf_own_monitor_.parent().c_str(), tf_own_monitor_.child().c_str());
            } else if (!static_map_odom_sent_.exchange(true)) {
                geometry_msgs::msg::TransformStamped identity;
                identity.header.stamp = this->get_clock()->now();
                if (invert_tf) {
                    identity.header.frame_id = odom_id;
                    identity.child_frame_id  = map_id;
                } else {
                    identity.header.frame_id = map_id;
                    identity.child_frame_id  = odom_id;
                }
                identity.transform.rotation.w = 1.0;
                // Static whitelist entry: /tf_static is transient_local, so
                // this exact stamp can be re-delivered arbitrarily late — it
                // must never age out of the self-stamp ring (tf_ownership.h).
                tf_own_monitor_.notePublishedStatic(
                    rclcpp::Time(identity.header.stamp).nanoseconds());
                static_br_->sendTransform(identity);
            }
        }
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
        // Lag the path tip like the map deskew (HARDENING §6.3): the tip pose
        // is what pubOdom composes into the map->odom TF, so it must come from
        // knots the estimator has finished refining (IEKF updates only the
        // last 4 RCPs; est_window resends only the last 5 knots — beyond that
        // every knot is final). spl_window_st_ns alone leaves the tip inside
        // the still-converging zone.
        const int64_t tip_lag_ns = map_deskew_lag_knots_ *
            std::max<int64_t>(spline_active_.getKnotTimeIntervalNs(), 0);
        const int64_t tip_limit_ns =
            std::min(spl_window_st_ns, spline_active_.maxTimeNs() - tip_lag_ns);
        while (path_t_ns_ < tip_limit_ns) {
            Eigen::Quaterniond orient_interp;
            Eigen::Vector3d t_interp = spline_active_.itpPosition(path_t_ns_);
            spline_active_.itpQuaternion(path_t_ns_, &orient_interp);
            opt_old_path.poses.push_back(CommonUtils::pose2msg(map_id, path_t_ns_, t_interp, orient_interp));
            path_t_ns_ += 100000000LL;  // 100 ms; integer to avoid double rounding of absolute-ns stamps
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

// Real entry point, compiled once into libresple. The thin Mapping_main.cpp
// wrapper (the executable's only TU) just forwards to this.
int mappingMain(int argc, char** argv) {
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
    // Use the bool return + log on failure rather than assert(): assert is a
    // no-op under -DNDEBUG (workspace default), so the original code silently
    // ignored a missing 'lidars' parameter and proceeded with empty names.
    if (!temp_nh->get_parameter({"lidars"}, lidar_names)) {
        RCLCPP_WARN(temp_nh->get_logger(),
            "Mapping: 'lidars' parameter not loadable; defaulting to empty (single anonymous lidar will be used).");
    }
    // LidarConfig ctor calls .at(N) on q_lb / t_lb vectors and will throw
    // std::out_of_range on truncated YAML. Without try/catch the throw
    // propagates through main() → std::terminate → SIGABRT, with the only
    // backtrace from the crash handler. Catch here so the failure is logged
    // with the offending lidar name.
    try {
        if (lidar_names.empty()) {
            lidars.emplace_back(temp_nh->get_node_parameters_interface(), "");
        } else {
            for (const auto& lidar_name : lidar_names) {
                lidars.emplace_back(temp_nh->get_node_parameters_interface(), lidar_name + ".");
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(temp_nh->get_logger(),
            "Mapping: LidarConfig construction failed: %s. "
            "Check 'lidars' / per-lidar params (topic_lidar, lidar_type, q_lb=[w,x,y,z], t_lb=[x,y,z]). Exiting.",
            e.what());
        rclcpp::shutdown();
        return 4;
    }
    // Create callback group for sensor processing
    auto sensor_cb_group = temp_nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    std::vector<MappingBase<pcl::PointXYZINormal>*> buffs;
    try {
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
            } else if (!lidar.type.compare("PointCloud2")) {
                buffs.push_back(new GenericPC2Buff(temp_nh, lidar, sensor_cb_group));
            } else {
                // exit(1) bypassed the crash handler and gave no log of why.
                RCLCPP_FATAL(temp_nh->get_logger(),
                    "Mapping: unknown lidar type '%s'. Supported: Ouster, Mid70Avia, HAP360, "
                    "AviaResple, Hesai, Mid360Boxi, PointCloud2 (generic). Exiting.", lidar.type.c_str());
                for (auto* buff : buffs) delete buff;
                rclcpp::shutdown();
                return 5;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(temp_nh->get_logger(),
            "Mapping: per-lidar buffer construction threw: %s. Exiting.", e.what());
        for (auto* buff : buffs) delete buff;
        rclcpp::shutdown();
        return 6;
    }
    
    // Lifecycle node initialization
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Mapping>(options, buffs);
    RCLCPP_INFO(node->get_logger(), "Mapping LifecycleNode created");

    // Transition to configured state. Wrap each lifecycle call so an exception
    // thrown inside on_configure / on_activate is logged here instead of
    // silently leaving the node in Unconfigured/ErrorProcessing while the
    // executor spins on a half-init node — which is the most plausible cause
    // of "no backtrace" startup crashes (the eventual SIGSEGV from a null
    // publisher / subscription has no useful prior log).
    {
        auto state = node->configure();
        const auto label = state.label();
        RCLCPP_INFO_STREAM(node->get_logger(),
            "Mapping configure() returned id=" << static_cast<int>(state.id())
            << " label=" << label);
        if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_FATAL_STREAM(node->get_logger(),
                "Mapping configure() failed (state=" << label
                << "); refusing to activate. Exiting non-zero.");
            for (auto buff : buffs) delete buff;
            rclcpp::shutdown();
            return 2;
        }
    }

    // Transition to active state (starts processing)
    {
        auto state = node->activate();
        const auto label = state.label();
        RCLCPP_INFO_STREAM(node->get_logger(),
            "Mapping activate() returned id=" << static_cast<int>(state.id())
            << " label=" << label);
        if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_FATAL_STREAM(node->get_logger(),
                "Mapping activate() failed (state=" << label
                << "); the executor would have spun on a half-init node. Exiting non-zero.");
            for (auto buff : buffs) delete buff;
            rclcpp::shutdown();
            return 3;
        }
    }
    RCLCPP_INFO(node->get_logger(), "Mapping active - entering executor spin");

    // Overload hardening (2026-07-07): mirror of the RESPLE node's
    // executor_threads. 0 (default) = rclcpp's one-thread-per-core.
    const int executor_threads = CommonUtils::readParam<int>(
        node->get_node_parameters_interface(), "executor_threads", 0);
    if (executor_threads > 0) {
        RCLCPP_INFO(node->get_logger(),
            "MultiThreadedExecutor limited to %d threads (executor_threads)",
            executor_threads);
    }
    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(),
        static_cast<size_t>(std::max(0, executor_threads)));
    exec.add_node(node->get_node_base_interface());
    exec.add_node(temp_nh);
    exec.spin();
    
    // Cleanup on shutdown (only if context still valid)
    bool sigint_path = !rclcpp::ok();
    if (!sigint_path) {
        RCLCPP_INFO(node->get_logger(), "Gracefully shutting down Mapping...");
        node->deactivate();
        node->cleanup();
        node->shutdown();
        exec.remove_node(node->get_node_base_interface());
        exec.remove_node(temp_nh);
        for (auto buff : buffs) {
            delete buff;
        }
        rclcpp::shutdown();
        return 0;
    }

    // SIGINT path: same rationale as RESPLE.cpp main(). Run on_shutdown
    // to stop the worker thread, then bail out with _Exit before any of
    // the wedge-prone teardown paths (executor, tf listener join, ROS
    // context shutdown, static dtors).
    RCLCPP_WARN(node->get_logger(), "Context invalid, forcing shutdown...");
    node->on_shutdown(node->get_current_state());
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(0);
}
