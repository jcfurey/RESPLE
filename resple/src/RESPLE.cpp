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

#include <algorithm>

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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
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
#include "estimate_msgs/msg/diagnostics.hpp"
#include "estimate_msgs/action/save_map.hpp"
#include "Estimator.h"
#include "utils/point_cloud_ingest.h"
#include "utils/filter_health.h"
#include "utils/overload_control.h"
#include "utils/tf_ownership.h"
#include "utils/dense_pub.h"
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
        // Reset the transform AND its latch together: on_cleanup also clears
        // the latch (hazard 71), but a transition-callback error routes
        // ErrorProcessing → Unconfigured WITHOUT running on_cleanup — a stale
        // true latch here would then skip the re-lookup and fuse the IMU
        // through this zeroed transform. Resetting both in one place makes
        // every path to a configured node consistent by construction.
        imu_to_baselink_ = geometry_msgs::msg::TransformStamped();
        have_imu_transform_ = false;
        
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

        // Overload hardening (2026-07-07): clamp the OpenMP width to what the
        // machine can actually give without starving the ROS executor and the
        // async map-update thread. Every shipped config hardcodes num_threads
        // for a big machine; on a 4-6 core box that oversubscribes the CPU and
        // the worker loses its cores mid-IEKF. Deliberate default-behaviour
        // change on small machines (< num_threads+2 cores) — see CLAUDE.md.
        {
            const unsigned hc = std::thread::hardware_concurrency();
            if (hc > 0) {
                const int cap = std::max(1, static_cast<int>(hc) - 2);
                if (num_threads_ > cap) {
                    RCLCPP_WARN(this->get_logger(),
                        "num_threads=%d exceeds this machine's budget (%u cores - 2 "
                        "reserved for executor/map thread); clamping to %d",
                        num_threads_, hc, cap);
                    num_threads_ = cap;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Using %d threads for parallel processing", num_threads_);
        RCLCPP_INFO(this->get_logger(), "Using %d nearest neighbor points for matching", num_match_points_);
        
        // Setup diagnostics. Register ONCE: diagnostic_updater::Updater::add
        // appends unconditionally, so registering here on every on_configure
        // stacks a duplicate "System Health" task each configure→cleanup→
        // configure cycle (all invoked per update). The one-shot guard keeps
        // the single registration across re-cycles; the callback reads only
        // atomics/caches, so it is safe in any lifecycle state. (Mapping
        // sidesteps this by registering in its constructor.)
        if (!diag_registered_) {
            diagnostics_.setHardwareID("RESPLE");
            diagnostics_.add("System Health", this, &RESPLE::updateDiagnostics);
            diag_registered_ = true;
        }
        
        // Initialize diagnostic metrics
        last_process_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
        frame_count_.store(0, std::memory_order_relaxed);
        total_computation_time_us_.store(0, std::memory_order_relaxed);
        total_iekf_iterations_.store(0, std::memory_order_relaxed);
        
        readParameters();
        
        // Create callback groups to separate sensor IO from control/estimation callbacks
        sensor_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // Create publishers (inactive until activated)
        // Depth sized to the Mapping subscriber's application queue
        // (kEstWindowQueueCap = 2000 in Mapping.cpp — keep the two in sync):
        // RELIABLE bounds loss only up to the WRITER's history depth, so the
        // old KEEP_LAST(50) meant a Mapping stall beyond ~2.5-5 s (est_window
        // publishes once per processed scan, ~10-20 Hz — NOT per knot)
        // dropped est_windows at the writer that the replica cannot recover,
        // skewing its knot time axis. est_window is the Mapping node's sole
        // input and is loss-sensitive; 2000 msgs ≈ 2-3 MB writer history.
        pub_est = this->create_publisher<estimate_msgs::msg::Estimate>("est_window", rclcpp::QoS(2000).reliable());
        // HARDENING Phase 4: consolidated, typed estimator diagnostics for
        // plotting (Foxglove / PlotJuggler). Relative topic so the production
        // namespace yields /localization/resple/resple_diagnostics; the
        // string-keyed diagnostic_updater output on /diagnostics remains for
        // aggregation. ~20 Hz while data flows — keep the queue shallow.
        pub_diag_ = this->create_publisher<estimate_msgs::msg::Diagnostics>(
            "resple_diagnostics", rclcpp::QoS(10).best_effort());
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
        // Latched base->sensor extrinsic frames (publish_extrinsic_tf, dliio
        // lesson 2026-07-10). Created only when enabled so the /tf_static
        // publisher doesn't exist otherwise; readParameters() ran above.
        if (publish_extrinsic_tf_) {
            static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        }
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
        pub_diag_->on_activate();
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
            } else if (!lidar.type.compare("PointCloud2")) {
                // Generic path: runtime PointField introspection, no per-sensor
                // point struct required. Field/unit overrides come from the
                // per-lidar YAML (time_field / time_unit / intensity_field).
                generic_adapter_cfg_ = resple::makeAdapterConfig(
                    lidar.time_field, lidar.time_unit, lidar.intensity_field);
                sub_generic = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, lidar_qos, std::bind(&RESPLE::genericLidarCallback, this, std::placeholders::_1), sensor_sub_opt);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "[RESPLE] unknown lidar type '%s' — no subscription created. "
                    "Supported: Ouster, Mid70Avia, HAP360, AviaResple, Hesai, "
                    "Mid360Boxi, PointCloud2 (generic).", lidar.type.c_str());
            }
        }
        
        // TF ownership guard: raw /tf + /tf_static watchers (absolute topic
        // names — tf2_ros broadcasters publish there regardless of namespace).
        // QoS mirrors the tf2_ros listener defaults (KeepLast(100); static is
        // transient_local so a latched foreign static transform on our pair is
        // seen even if it was published before we activated). Default callback
        // group: the callback is O(frames) string compares, no heavy work.
        //
        // Only subscribe when the guard can actually act: we OWN the pair
        // (publish_tf → conflict detection / yield), or the absence check is
        // armed (publish_tf=false but tf_absent_warn_sec>0 → "external owner
        // expected"). A demoted node with the absence check off has nothing to
        // defend and nothing to watch for — skip the /tf processing entirely.
        if (publish_tf || tf_absent_warn_sec_ > 0.0) {
            sub_tf_watch_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/tf", rclcpp::QoS(rclcpp::KeepLast(100)),
                [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { tfWatchCallback(msg, false); });
            sub_tf_static_watch_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/tf_static", rclcpp::QoS(rclcpp::KeepLast(100)).transient_local(),
                [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { tfWatchCallback(msg, true); });
        }
        tf_absence_wait_.reset();  // window (re)starts at first valid clock sample

        // Start processing thread. The exited-flag store is the lambda's last
        // action so joinProcessingThreadBounded can tell "worker finished,
        // join() will return promptly" from "worker wedged, must detach"
        // without a second thread touching processing_thread_ (see there).
        processing_active_ = true;
        processing_thread_exited_.store(false, std::memory_order_release);
        processing_thread_ = std::thread([this] {
            processData();
            processing_thread_exited_.store(true, std::memory_order_release);
        });

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
        pub_diag_->on_deactivate();
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
        sub_generic.reset();
        sub_tf_watch_.reset();
        sub_tf_static_watch_.reset();

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

        // A map-update future abandoned in waitForMapUpdateBounded (>2s stall)
        // leaves map_update_future_ moved-from (invalid) while the zombie
        // lambda may still be running. Reset the (already-invalid or
        // already-completed) future as hygiene, but deliberately do NOT clear
        // map_update_pending_: only the lambda itself clears it (release-store
        // as its last act), and the worker treats pending+invalid as "zombie
        // alive — skip map updates" (see processData). Force-clearing here
        // would let the next activation swap the bg buffers and launch a
        // second lambda while the zombie still reads them, and the zombie's
        // trailing store(false) would then mask the NEW lambda (2026-07-14
        // review; supersedes the earlier hazard-76 reset).
        map_update_future_ = std::future<void>{};

        // Clear buffers and data structures. `lidars` is iterated by
        // lasermapFovSegment inside the async map-update lambda under mtx_map_;
        // if that lambda was abandoned above and is still running, destroying
        // the container here would invalidate its iterator (UAF). Serialize via
        // mtx_map_ — but with a BOUNDED acquisition: a lambda wedged INSIDE its
        // mtx_map_ hold (the canonical abandonment cause) would otherwise hang
        // this lifecycle callback forever, defeating the bounded-teardown
        // design the 2s waits above implement. On timeout, leak the containers
        // (leak-but-safe; the zombie holds the lock, so nothing else can touch
        // them either) and log loudly.
        {
            std::unique_lock<std::shared_mutex> map_lock(mtx_map_, std::defer_lock);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (!map_lock.try_lock()) {
                if (std::chrono::steady_clock::now() >= deadline) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (map_lock.owns_lock()) {
                lidars.clear();
                lidars_data.clear();
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "[RESPLE] mtx_map_ still held (wedged map update?) after 2s; "
                    "leaking lidars/lidars_data to keep cleanup bounded");
            }
        }
        imu_buff.clear();
        imu_meas.clear();
        pt_meas.clear();
        pc_world.clear();
        accum_nearest_points.clear();
        map_insert_staging_.clear();
        released_w_.clear();
        knot_rot_checked_ = 0;
        if_init_filter = false;
        if_init_map = false;
        localmap_initialized_ = false;
        // Boxes recorded against the PREVIOUS activation's cube must not be
        // carried into a freshly-initialised one — they would be re-submitted
        // to Delete_Point_Boxes against an unrelated local map.
        cub_needrm.clear();
        radius_prune_initialized_ = false;
        nis_hold_active_.store(false);
        imu_first_logged_.store(false);
        imu_count_logged_.store(false);
        lidar_first_logged_.store(false);
        init_complete_logged_.store(false);
        iekf_first_logged_.store(false);
        est_window_first_logged_.store(false);
        // A re-cycled node re-initializes its spline on a fresh timeline; a
        // stale duplicate-suppression watermark would silently gate pose/odom/
        // TF until the new timeline overtakes the old one (finding #12). The
        // LO IMU-wait and frame-id state must restart with it (findings #9/#10).
        last_pose_pub_time_ns_ = std::numeric_limits<int64_t>::min();
        lo_imu_wait_.reset();
        init_attitude_wait_.reset();
        init_attitude_delta_deg_.store(-1.0, std::memory_order_relaxed);
        imu_frame_id_.clear();
        // The IMU extrinsic is re-looked-up per activation only while
        // have_imu_transform_ is false, but that latch is a node member that
        // survives cleanup — while on_configure resets imu_to_baselink_ to a
        // zero-quaternion default. Without clearing the latch here, a
        // deactivate→cleanup→configure→activate re-cycle skips the re-lookup
        // and fuses the IMU (and gravity init) through that bogus extrinsic.
        have_imu_transform_ = false;

        // Reset publishers
        pub_est.reset();
        pub_diag_.reset();
        pub_start_time.reset();
        pub_pose.reset();
        pub_pose_cov.reset();
        pub_odom.reset();
        pub_cur_scan.reset();
        br.reset();
        static_br_.reset();

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

    // Bounded join, used during lifecycle teardown so that a wedged ikd-Tree
    // (rebuild thread holding internal locks) can't hold up shutdown past the
    // launcher's 5-second SIGINT→SIGTERM grace window.
    //
    // Implementation note: this used to dispatch join() onto a std::async
    // task and detach() from this thread on timeout. That raced two threads
    // on the SAME std::thread object — when the worker exited just after the
    // deadline, the async join() consumed the thread id while this thread
    // called detach() on it (UB; TSan's runtime CHECK-aborted on it during
    // the Phase 3.1 sweep). The async future's destructor also blocked until
    // join() returned, so the join wasn't actually bounded. Instead, poll the
    // worker's exited flag (its last store before returning): once set,
    // join() is guaranteed to return promptly; until then nobody touches
    // processing_thread_, so the timeout detach() is single-threaded.
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
            // processing_active_ is already false; the worker is just
            // wedged on a third-party lock and will tear down with the
            // process. Detach so shutdown can proceed.
            processing_thread_.detach();
            RCLCPP_WARN(this->get_logger(),
                "processing thread did not exit within timeout; detached");
        }
    }

    void waitForMapUpdateBounded(std::chrono::milliseconds timeout)
    {
        if (!map_update_future_.valid()) return;
        if (map_update_future_.wait_for(timeout) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(),
                "background map-update did not finish within timeout; "
                "abandoning to avoid blocking shutdown");
            // Park in a thread-local LIST to keep the future alive (and the
            // task running to completion) without making the lifecycle
            // callback wait on it. A list, not a single slot: overwriting a
            // single slot destroys the previously-parked future, and a
            // std::async future's destructor JOINS its task — a second
            // abandonment would block unboundedly on the first zombie
            // (2026-07-14 review). Reap finished zombies first (their
            // destructors are instant), then append.
            static thread_local std::vector<std::future<void>> abandoned;
            abandoned.erase(
                std::remove_if(abandoned.begin(), abandoned.end(),
                    [](std::future<void>& f) {
                        return f.wait_for(std::chrono::seconds(0)) ==
                               std::future_status::ready;
                    }),
                abandoned.end());
            abandoned.push_back(std::move(map_update_future_));
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
                    // Node clock can jump backwards (bag loop restart under
                    // use_sim_time): re-arm instead of going silent until the
                    // clock re-reaches the stale stamp.
                    if (now_clk < hb_last_log) hb_last_log = now_clk;
                    if ((now_clk - hb_last_log).seconds() >= 2.0) {
                        hb_last_log = now_clk;
                        size_t pc_buff_total = 0, pt_buff_total = 0;
                        for (auto& [n, d] : lidars_data) {
                            {
                                // pc_buff is callback-written: size() reads the
                                // deque internals, so it needs mtx_pc like the
                                // diagnostics block above (finding #28).
                                std::lock_guard<std::mutex> lk(d.mtx_pc);
                                pc_buff_total += d.pc_buff.size();
                            }
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
                        // Real-time factor: spline data-time advance / wall time
                        // over this ~2 s window. Only meaningful when the IEKF
                        // ran in the window (else an idle/paused bag reads as 0).
                        // With latency shedding active this stays ~1 by design —
                        // read together with shed_scans.
                        {
                            const auto now_mono = std::chrono::steady_clock::now();
                            if (rtf_prev_spline_max_ > 0 && spline_max > 0 &&
                                hb_iekf_count > 0) {
                                const double wall_s = std::chrono::duration<double>(
                                    now_mono - rtf_prev_wall_).count();
                                if (wall_s > 0.5) {
                                    const double data_s =
                                        double(spline_max - rtf_prev_spline_max_) / 1e9;
                                    rt_factor_.store(static_cast<float>(data_s / wall_s),
                                                     std::memory_order_relaxed);
                                }
                            }
                            rtf_prev_spline_max_ = spline_max;
                            rtf_prev_wall_ = now_mono;
                        }
                        // TF ownership absence check: demoted
                        // (odom/publish_tf=false, external owner expected —
                        // e.g. the robot_localization odom EKF) but the pair
                        // is not being published. Two distinct failures:
                        // never wired up (one-shot after the grace window),
                        // and owner DIED MID-RUN (lastPairSeen goes stale;
                        // re-arms whenever the owner resumes, so each outage
                        // warns once). Both leave downstream TF consumers
                        // silently starving.
                        if (!publish_tf && tf_absent_warn_sec_ > 0) {
                            const int64_t last_seen = tf_own_monitor_.lastPairSeenNs();
                            if (last_seen != tf_last_pair_seen_ns_) {
                                // Owner is alive (pair observed since last
                                // check): re-arm the outage warning.
                                tf_last_pair_seen_ns_ = last_seen;
                                tf_absence_warned_.store(false, std::memory_order_relaxed);
                            }
                            if (!tf_absence_warned_.load(std::memory_order_relaxed)) {
                                if (last_seen == 0) {
                                    const double waited = tf_absence_wait_.elapsedSec(
                                        this->now().nanoseconds());
                                    if (waited > tf_absent_warn_sec_) {
                                        tf_absence_warned_.store(true, std::memory_order_relaxed);
                                        RCLCPP_WARN(this->get_logger(),
                                            "[RESPLE] odom/publish_tf=false but NO ONE has published "
                                            "%s->%s in %.0f s. The external owner (odom EKF?) is not "
                                            "wired up - TF consumers downstream are starving. Either "
                                            "start it or set odom/publish_tf: true.",
                                            tf_own_monitor_.parent().c_str(),
                                            tf_own_monitor_.child().c_str(), waited);
                                    }
                                } else {
                                    const double stale_s =
                                        (this->now().nanoseconds() - last_seen) / 1e9;
                                    if (stale_s > tf_absent_warn_sec_) {
                                        tf_absence_warned_.store(true, std::memory_order_relaxed);
                                        RCLCPP_WARN(this->get_logger(),
                                            "[RESPLE] the external owner of %s->%s STOPPED "
                                            "publishing %.0f s ago (died mid-run?). TF consumers "
                                            "downstream are starving on a stale transform.",
                                            tf_own_monitor_.parent().c_str(),
                                            tf_own_monitor_.child().c_str(), stale_s);
                                    }
                                }
                            }
                        }
                        bool map_busy = map_update_pending_.load(std::memory_order_acquire)
                            && map_update_future_.valid()
                            && map_update_future_.wait_for(std::chrono::seconds(0))
                                != std::future_status::ready;
                        RCLCPP_INFO(this->get_logger(),
                            "[RESPLE] heartbeat: loops=%zu iekf=%zu coll_false=%zu "
                            "pc_buff=%zu pt_buff=%zu imu_int=%zu "
                            "spline_knots=%ld spline_max=%ld map_busy=%d "
                            "rtf=%.2f shed=%lu overruns=%u",
                            hb_loop_count, hb_iekf_count, hb_collect_false_count,
                            pc_buff_total, pt_buff_total, imu_buff_size,
                            spline_n, spline_max, int(map_busy),
                            rt_factor_.load(std::memory_order_relaxed),
                            static_cast<unsigned long>(shed_scans_.load(std::memory_order_relaxed)),
                            cycle_overruns_.load(std::memory_order_relaxed));
                        hb_loop_count = hb_iekf_count = hb_collect_false_count = 0;
                    }
                }
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (true) {
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_frame(new pcl::PointCloud<pcl::PointXYZINormal>());
                    int64_t time_begin;
                    {
                        std::lock_guard<std::mutex> lock(lidar_data.mtx_pc);
                        // Check emptiness UNDER mtx_pc. This used to be the
                        // while-condition (`while (!t_buff.empty())`) read outside
                        // the lock, racing with the sensor callback's locked
                        // push_back on the same deque (TSan-flagged data race on
                        // the deque internals — same class as the Phase 1.5 IMU
                        // imu_int_buff fix).
                        if (lidar_data.t_buff.empty()) break;
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
            // Cycle-budget overrun accounting starts AFTER initialization so
            // the deliberate 50 ms init-wait sleeps above never count.
            const auto cycle_t0 = std::chrono::steady_clock::now();
            bool collected_any = false;
            while (true) {
                // Phase 4 stage timing: drain covers THIS frame's collection
                // (captured per inner iteration — several frames can process
                // per outer loop pass).
                const auto drain_start = std::chrono::high_resolution_clock::now();
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
                // Track computation time (Phase 4 also splits per-stage).
                auto frame_start = std::chrono::high_resolution_clock::now();
                const float drain_ms = std::chrono::duration<float, std::milli>(frame_start - drain_start).count();

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
                        total_iekf_iterations_.fetch_add(estimator_lo.lastIterations(), std::memory_order_relaxed);
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
                        total_iekf_iterations_.fetch_add(estimator_lio.lastIterations(), std::memory_order_relaxed);
                        // Consume-once. pt_meas is cleared after every frame
                        // (below, after deskew); imu_meas was not, and the trim
                        // above only drops samples older than one knot interval
                        // — so every sample still inside that window was fused
                        // AGAIN on the next pass. That happens whenever a knot
                        // interval needs more than one batch, i.e. whenever the
                        // interval holds more than num_points_upd points: the
                        // same 6 IMU rows enter the information sum twice, with
                        // a residual that the first fusion already absorbed. The
                        // state barely moves, but the posterior tightens as
                        // though a second, independent sample had arrived —
                        // double-counted information, i.e. an over-confident
                        // covariance on exactly the directions the IMU
                        // observes, published straight into the odom EKF.
                        //
                        // Clear only when the batch was actually folded in: a
                        // frame that found zero correspondences never called
                        // update(), so its samples are still unfused and must
                        // survive to the next pass (subject to the trim).
                        if (estimator_lio.lastUpdatePerformed()) {
                            imu_meas.clear();
                        }
                    }
                }
                const auto iekf_end = std::chrono::high_resolution_clock::now();
                {
                    // HARDENING §3.2: accumulate this update's correspondence
                    // funnel into the cumulative atomics consumed by
                    // updateDiagnostics (executor thread).
                    const Association::CorrespStats& cs = if_lidar_only
                        ? estimator_lo.corresp_stats : estimator_lio.corresp_stats;
                    corresp_candidates_.fetch_add(cs.candidates, std::memory_order_relaxed);
                    corresp_passed_knn_.fetch_add(cs.passed_knn, std::memory_order_relaxed);
                    corresp_passed_plane_.fetch_add(cs.passed_plane, std::memory_order_relaxed);
                    corresp_used_.fetch_add(cs.used, std::memory_order_relaxed);
                }
                {
                    // HARDENING §3.3: feed the windowed NIS consistency
                    // detector with the final IEKF iteration's innovation
                    // statistic (NaN when the update was skipped — counted as
                    // a breach). Worker-thread state; verdicts exported via
                    // atomics for the executor-thread diagnostics.
                    double nis_v;
                    int nis_dof;
                    if (if_lidar_only) {
                        nis_v = estimator_lo.lastNis();
                        nis_dof = estimator_lo.lastNisDof();
                    } else {
                        nis_v = estimator_lio.lastNis();
                        nis_dof = estimator_lio.lastNisDof();
                    }
                    const auto fh_state = nis_detector_.feed(nis_v, nis_dof);
                    filter_health_state_.store(static_cast<int>(fh_state), std::memory_order_relaxed);
                    nis_window_mean_.store(nis_detector_.lastWindowMean(), std::memory_order_relaxed);
                    if (fh_state == resple::health::FilterState::DIVERGED) {
                        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "[RESPLE] NIS divergence detected: windowed NIS mean %.2fx dof "
                            "(consistent filter expects ~1.0). The filter is over-confident "
                            "or correspondences are inconsistent — pose covariance is no "
                            "longer trustworthy.", nis_detector_.lastWindowMean());
                    }
                    // HARDENING §3.3 recovery policy (nis_recovery_mode param).
                    switch (nis_recovery_mode_) {
                    case NisRecoveryMode::OFF:
                        break;
                    case NisRecoveryMode::HOLD:
                        // Latch the hold on DIVERGED; release only on a full
                        // OK verdict (the detector's hysteresis), so WARN
                        // during recovery keeps the gate closed.
                        if (fh_state == resple::health::FilterState::DIVERGED) {
                            if (!nis_hold_active_) {
                                RCLCPP_ERROR(this->get_logger(),
                                    "[RESPLE] nis_recovery_mode=hold: suspending odometry/TF "
                                    "publication until the NIS window recovers.");
                            }
                            nis_hold_active_ = true;
                        } else if (fh_state == resple::health::FilterState::OK && nis_hold_active_) {
                            nis_hold_active_ = false;
                            RCLCPP_WARN(this->get_logger(),
                                "[RESPLE] NIS window recovered; resuming odometry/TF publication.");
                        }
                        break;
                    case NisRecoveryMode::RESET:
                        if (fh_state == resple::health::FilterState::DIVERGED) {
                            // Reinflate the IEKF covariance to the recovery
                            // covariance (nis_reset_cov), keeping the state
                            // (spline + biases): an over-confident covariance is
                            // exactly what the NIS verdict detects, and a LARGE
                            // reinflation lets new measurements re-correct the
                            // state (bug A2 — the old path reset to the tiny
                            // startup prior, which deflated and froze the filter).
                            // Restart the detector window so the next verdict
                            // reflects the post-reset filter.
                            if (if_lidar_only) {
                                estimator_lo.reinflateCovariance();
                            } else {
                                estimator_lio.reinflateCovariance();
                            }
                            nis_detector_.reset();
                            nis_resets_.fetch_add(1, std::memory_order_relaxed);
                            RCLCPP_ERROR(this->get_logger(),
                                "[RESPLE] nis_recovery_mode=reset: IEKF covariance reinflated "
                                "to the recovery covariance nis_reset_cov (reset #%lu).",
                                static_cast<unsigned long>(nis_resets_.load(std::memory_order_relaxed)));
                        }
                        break;
                    }
                }
                size_t n_release = 0;
                Eigen::Quaterniond q_edge = Eigen::Quaterniond::Identity();
                {
                    // pointBodyToWorld reads spline; keep serialized with
                    // IEKF writes via spline_mutex_.
                    std::lock_guard<std::mutex> spline_lock(spline_mutex_);
                    // Edge orientation for the localizability diagnostic below
                    // (rotates world-frame plane normals into the body frame;
                    // one rotation for the whole update is adequate for a
                    // diagnostic over a ≤100 ms point window).
                    spline->itpQuaternion(spline->maxTimeNs(), &q_edge);
                    if (map_insert_lag_knots_ == 0) {
                        // Upstream behavior: world-fix at IEKF time (edge knots).
                        #pragma omp parallel for num_threads(num_threads_)
                        for (size_t i = 0; i < pt_meas.size(); i++) {
                            PointData& pt_data = pt_meas[i];
                            Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w, pt_data.t_bl, pt_data.q_bl);
                        }
                    } else {
                        // §6.3 insertion lag: stage body-frame, then deskew and
                        // release everything past the convergence horizon with
                        // knot values the IEKF has finished refining. pt_meas
                        // is time-sorted and cycles are monotone, so the
                        // staging deque is time-ordered and the release scan
                        // can stop at the first too-new entry.
                        for (size_t i = 0; i < pt_meas.size(); i++) {
                            PointData& pt_data = pt_meas[i];
                            map_insert_staging_.push_back(StagedMapPoint{
                                pt_data.time_ns, pt_data.pt, pt_data.q_bl,
                                pt_data.t_bl, std::move(pt_neighbors_[i])});
                        }
                        int64_t release_horizon_ns = spline->maxTimeNs() -
                            map_insert_lag_knots_ * spline->getKnotTimeIntervalNs();
                        if (map_insert_cov_gate_deg_ > 0.0) {
                            // VoxelMap-style uncertainty convergence: if the
                            // edge pose is already tight, the hold buys
                            // nothing — release everything staged.
                            const double ori_var_tr = (if_lidar_only
                                ? estimator_lo.getLastPoseCovariance()
                                : estimator_lio.getLastPoseCovariance())
                                    .block<3, 3>(3, 3).trace();
                            const double ori_std_deg =
                                std::sqrt(std::max(ori_var_tr, 0.0)) * 180.0 / M_PI;
                            if (ori_std_deg < map_insert_cov_gate_deg_) {
                                release_horizon_ns = spline->maxTimeNs();
                            }
                        }
                        while (n_release < map_insert_staging_.size() &&
                               map_insert_staging_[n_release].time_ns <= release_horizon_ns) {
                            n_release++;
                        }
                        released_w_.resize(n_release);
                        #pragma omp parallel for num_threads(num_threads_)
                        for (size_t i = 0; i < n_release; i++) {
                            StagedMapPoint& s = map_insert_staging_[i];
                            Association::pointBodyToWorld(s.time_ns, spline, s.pt, released_w_[i], s.t_bl, s.q_bl);
                        }
                    }
                    // §6.3 knot under-resolution check: each knot's FINAL
                    // ort_delta norm is the rotation absorbed in one knot
                    // interval; values near/above the threshold mean knot_hz
                    // under-fits the motion (Coco-LIC lesson) and trailing-
                    // edge jitter is expected. Checked once per knot as it
                    // leaves the est window (>= 5 behind the edge => final).
                    if (knot_rotation_warn_rad_ > 0.0) {
                        const int64_t total_k = spline->totalKnots();
                        const int64_t pruned_k = spline->numKnotsPruned();
                        if (knot_rot_checked_ < pruned_k) knot_rot_checked_ = pruned_k;
                        for (; knot_rot_checked_ < total_k - 5; ++knot_rot_checked_) {
                            const double rot = spline->getKnotOrtDel(
                                static_cast<size_t>(knot_rot_checked_ - pruned_k)).norm();
                            const uint32_t mrad = static_cast<uint32_t>(rot * 1000.0);
                            uint32_t prev = knot_rot_max_mrad_.load(std::memory_order_relaxed);
                            while (mrad > prev &&
                                   !knot_rot_max_mrad_.compare_exchange_weak(prev, mrad)) {}
                            if (rot > knot_rotation_warn_rad_) {
                                knot_rot_warn_count_.fetch_add(1, std::memory_order_relaxed);
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "[RESPLE] rotation per knot %.3f rad exceeds %.3f (knot #%ld): "
                                    "knot_hz under-resolves this motion — expect trailing-edge "
                                    "jitter; consider a higher knot_hz.",
                                    rot, knot_rotation_warn_rad_,
                                    static_cast<long>(knot_rot_checked_));
                            }
                        }
                    }
                    // Cache knot count for updateDiagnostics (may run on the
                    // executor thread; atomic read there is lock-free).
                    cached_spline_knots_.store(spline->numKnots(), std::memory_order_relaxed);
                    cached_spline_total_knots_.store(spline->totalKnots(), std::memory_order_relaxed);
                }
                // §3.2/§6.3 localizability diagnostic from this update's valid
                // point-to-plane constraints (rationale at the member decls:
                // separate 3×3 blocks, per-point normalized, report-only).
                {
                    Eigen::Matrix3d E_tt = Eigen::Matrix3d::Zero();
                    Eigen::Matrix3d E_rr = Eigen::Matrix3d::Zero();
                    size_t n_valid = 0;
                    for (size_t i = 0; i < pt_meas.size(); i++) {
                        const PointData& pt_data = pt_meas[i];
                        if (!pt_data.if_valid) continue;
                        const Eigen::Vector3d n_b = q_edge.conjugate() * pt_data.normvec;
                        const Eigen::Vector3d rxn = pt_data.pt_b.cross(n_b);
                        E_tt.noalias() += n_b * n_b.transpose();
                        E_rr.noalias() += rxn * rxn.transpose();
                        n_valid++;
                    }
                    if (n_valid >= 6) {
                        E_tt /= static_cast<double>(n_valid);
                        E_rr /= static_cast<double>(n_valid);
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_t(E_tt);
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_r(E_rr);
                        const Eigen::Vector3d lt = es_t.eigenvalues();  // ascending
                        const Eigen::Vector3d lr = es_r.eigenvalues();
                        degen_min_eig_trans_.store(static_cast<float>(lt(0)), std::memory_order_relaxed);
                        degen_min_eig_rot_.store(static_cast<float>(lr(0)), std::memory_order_relaxed);
                        degen_cond_trans_.store(static_cast<float>(lt(0) > 0.0 ? lt(2) / lt(0) : 0.0), std::memory_order_relaxed);
                        degen_cond_rot_.store(static_cast<float>(lr(0) > 0.0 ? lr(2) / lr(0) : 0.0), std::memory_order_relaxed);
                    }
                    // Gate activity snapshot (worker context, like corresp_stats).
                    loc_gate_axes_diag_.store(if_lidar_only
                        ? estimator_lo.locGateAxes() : estimator_lio.locGateAxes(),
                        std::memory_order_relaxed);
                    loc_gate_updates_diag_.store(if_lidar_only
                        ? estimator_lo.locGateUpdateCount() : estimator_lio.locGateUpdateCount(),
                        std::memory_order_relaxed);
                    loc_gate_cov_infl_diag_.store(if_lidar_only
                        ? estimator_lo.locGateCovInflMaxStd() : estimator_lio.locGateCovInflMaxStd(),
                        std::memory_order_relaxed);
                }
                if (map_insert_lag_knots_ == 0) {
                    for (size_t i = 0; i < pt_meas.size(); i++) {
                        PointData& pt_data = pt_meas[i];
                        pc_world.points.push_back(pt_data.pt_w);
                        accum_nearest_points.push_back(std::move(pt_neighbors_[i]));
                    }
                } else {
                    for (size_t i = 0; i < n_release; i++) {
                        pc_world.points.push_back(released_w_[i]);
                        accum_nearest_points.push_back(std::move(map_insert_staging_.front().neighbors));
                        map_insert_staging_.pop_front();
                    }
                    released_w_.clear();
                }
                pt_meas.clear();
                const auto deskew_end = std::chrono::high_resolution_clock::now();
                // max_spl_knots tracks totalKnots() (monotonic, includes
                // pruned knots) so the growth gate keeps firing once Phase 3.1
                // pruning caps numKnots() at the retention window.
                // publish_est_window=false (overload hardening) skips the msg
                // build + reliable-QoS publish entirely — ONLY safe with the
                // Mapping node disabled (use_mapping:=false): /est_window is
                // Mapping's sole input and it starves silently without it.
                if (publish_est_window_ && spline->totalKnots() > max_spl_knots) {
                    estimate_msgs::msg::Spline spline_msg;
                    // Estimate.pose_covariance is DOCUMENTED in the .msg as the
                    // 6x6 IEKF posterior for the last knot, but was never
                    // populated — it shipped 36 zeros, i.e. PERFECT CERTAINTY,
                    // to any consumer that trusted the field. Fill it (and the
                    // header, which shipped stamp 0) from the same posterior
                    // /odom publishes. Computed under spline_mutex_ because
                    // getLastPoseCovariance interpolates the spline.
                    Eigen::Matrix<double, 6, 6> P_last;
                    int64_t est_stamp_ns = 0;
                    {
                        std::lock_guard<std::mutex> spline_lock(spline_mutex_);
                        spline->getSplineMsg(spline_msg, std::max<int64_t>(max_spl_knots - 1, 0));
                        max_spl_knots = spline->totalKnots();
                        est_stamp_ns = spline->maxTimeNs();
                        P_last = if_lidar_only ? estimator_lo.getLastPoseCovariance()
                                               : estimator_lio.getLastPoseCovariance();
                    }
                    // Same on-the-wire guard as /odom: a non-finite posterior
                    // must not leave the node in ANY message.
                    if (resple::health::sanitizeCovariance(P_last).modified()) {
                        cov_sanitized_.fetch_add(1, std::memory_order_relaxed);
                    }
                    estimate_msgs::msg::Estimate est_msg;
                    // DATA time (spline edge), consistent with /odom and
                    // /current_scan — not the publish wall time.
                    est_msg.header.stamp = rclcpp::Time(est_stamp_ns);
                    est_msg.header.frame_id = odom_id;
                    est_msg.spline = spline_msg;
                    est_msg.if_full_window.data = (max_spl_knots >= 4);
                    est_msg.runtime.data = 0;
                    for (int r = 0; r < 6; ++r) {
                        for (int c = 0; c < 6; ++c) {
                            est_msg.pose_covariance[r * 6 + c] = P_last(r, c);
                        }
                    }
                    pub_est->publish(est_msg);
                    if (!est_window_first_logged_.exchange(true)) {
                        RCLCPP_INFO(this->get_logger(),
                            "[RESPLE] first /est_window published (numKnots=%ld)",
                            spline->numKnots());
                    }
                }
                // Phase 3.1 sliding-window knot pruning (hazard 4: unbounded
                // knot growth, measured live in the 2.6 data-path sweep).
                // Runs AFTER the est_window publish so every knot is on the
                // wire before it can be dropped, and under spline_mutex_ —
                // the same lock every other spline reader/writer takes. The
                // retention window is far wider than every backward-looking
                // consumer (IEKF: last 4 knots; est_window: last 5; deskew:
                // the current scan, ~10 knots at 100 Hz).
                if (spline_prune_keep_knots_ > 0) {
                    std::lock_guard<std::mutex> spline_lock(spline_mutex_);
                    spline->pruneFrontKnots(spline_prune_keep_knots_);
                }
                // Pose/odom/TF publish is INLINE and (normally) unconditional.
                // Keeping it out of the async-map-update lambda means the odom
                // frame keeps flowing even if the background map mutation is
                // wedged inside ikd-Tree (rebuild can stall on big trees,
                // and previously this also stalled processData itself
                // because the loop blocked on map_update_future_.wait()).
                //
                // HARDENING §3.3 'hold' recovery: while the NIS hold is
                // latched, the last published pose IS the held estimate —
                // downstream (the odom EKF) coasts on its other sources
                // instead of fusing an untrustworthy pose. est_window /
                // map updates continue (internal estimation keeps running so
                // the detector can observe recovery).
                if (nis_hold_active_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "[RESPLE] odometry/TF publication held (NIS divergence, "
                        "nis_recovery_mode=hold).");
                } else {
                    publishPoseAndTf();
                }

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
                    if (map_update_pending_.load(std::memory_order_acquire)
                        && !map_update_future_.valid()) {
                        // pending==true with an INVALID future means a prior
                        // lambda was abandoned by waitForMapUpdateBounded (>2s
                        // stall) and may STILL BE RUNNING — reachable via
                        // deactivate→activate, which skips on_cleanup's reset.
                        // Falling through to the swap would race its
                        // mapIncremental(pc_world_bg_, …); waiting is
                        // impossible (no future). Skip this cycle: the zombie
                        // clears pending as its last act, after which map
                        // updates resume cleanly (2026-07-14 review).
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "[RESPLE] abandoned map-update still pending; "
                            "skipping map update this cycle until it clears.");
                        continue;
                    }
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
                    // publish_current_scan=false (overload hardening) skips
                    // the per-point copy + serialization; visualization-only
                    // output, nothing downstream consumes it.
                    if (publish_current_scan_) {
                        publishCurrentScan(pc_world_bg_);
                    }
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
                            const auto map_upd_start = std::chrono::high_resolution_clock::now();
                            std::unique_lock<std::shared_mutex> map_lock(mtx_map_);
                            mapIncremental(pc_world_bg_, accum_nearest_points_bg_);
                            lasermapFovSegment();
                            last_map_update_us_.store(
                                std::chrono::duration_cast<std::chrono::microseconds>(
                                    std::chrono::high_resolution_clock::now() - map_upd_start).count(),
                                std::memory_order_relaxed);
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

                // HARDENING Phase 4: consolidated typed diagnostics, one
                // message per processed frame. Everything here is either a
                // worker-local value or a relaxed atomic snapshot — no locks
                // beyond the brief spline/cov reads below.
                {
                    estimate_msgs::msg::Diagnostics dmsg;
                    dmsg.header.stamp = rclcpp::Time(max_time_ns);
                    dmsg.header.frame_id = odom_id;
                    {
                        std::lock_guard<std::mutex> spline_lock(spline_mutex_);
                        dmsg.knot_count = spline->numKnots();
                        dmsg.knot_total = spline->totalKnots();
                        // 6x6 pose covariance at maxTimeNs (reads the spline
                        // for the interpolation Jacobian — hence the lock).
                        const Eigen::Matrix<double, 6, 6> pose_cov = if_lidar_only
                            ? estimator_lo.getLastPoseCovariance()
                            : estimator_lio.getLastPoseCovariance();
                        dmsg.cov_trace = pose_cov.trace();
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(pose_cov);
                        dmsg.cov_lambda_min = (es.info() == Eigen::Success)
                            ? es.eigenvalues().minCoeff()
                            : std::numeric_limits<double>::quiet_NaN();
                    }
                    dmsg.lidar_buffer_depth = cached_lidar_buf_.load(std::memory_order_relaxed);
                    dmsg.imu_buffer_depth = static_cast<int64_t>(imu_buff.size());
                    {
                        std::lock_guard<std::mutex> lock(m_buff);
                        dmsg.imu_staging_depth = static_cast<int64_t>(imu_int_buff.size());
                    }
                    const Association::CorrespStats& cs = if_lidar_only
                        ? estimator_lo.corresp_stats : estimator_lio.corresp_stats;
                    dmsg.corresp_candidates = cs.candidates;
                    dmsg.corresp_passed_window = cs.passed_window;
                    dmsg.corresp_passed_knn = cs.passed_knn;
                    dmsg.corresp_passed_plane = cs.passed_plane;
                    dmsg.corresp_used = cs.used;
                    if (if_lidar_only) {
                        dmsg.iekf_numerical_failures = estimator_lo.num_numerical_failures_.load(std::memory_order_relaxed);
                        dmsg.nis = estimator_lo.lastNis();
                        dmsg.nis_dof = estimator_lo.lastNisDof();
                    } else {
                        dmsg.iekf_numerical_failures = estimator_lio.num_numerical_failures_.load(std::memory_order_relaxed);
                        dmsg.nis = estimator_lio.lastNis();
                        dmsg.nis_dof = estimator_lio.lastNisDof();
                    }
                    dmsg.filter_state = static_cast<uint8_t>(filter_health_state_.load(std::memory_order_relaxed));
                    dmsg.recovery_hold_active = nis_hold_active_.load(std::memory_order_relaxed);
                    dmsg.recovery_resets = nis_resets_.load(std::memory_order_relaxed);
                    dmsg.deskew_out_of_range = Association::out_of_range_queries_.load(std::memory_order_relaxed);
                    dmsg.map_size = ikdtree.size();
                    dmsg.map_radius_prunes = radius_prune_ops_.load(std::memory_order_relaxed);
                    dmsg.dropped_scans = dropped_scans_.load(std::memory_order_relaxed);
                    dmsg.dropped_imu = dropped_imu_.load(std::memory_order_relaxed);
                    dmsg.drain_ms = drain_ms;
                    dmsg.iekf_ms = std::chrono::duration<float, std::milli>(iekf_end - frame_start).count();
                    dmsg.deskew_ms = std::chrono::duration<float, std::milli>(deskew_end - iekf_end).count();
                    dmsg.frame_total_ms = frame_duration.count() / 1000.0f;
                    dmsg.map_update_ms = last_map_update_us_.load(std::memory_order_relaxed) / 1000.0f;
                    // Overload / real-time health (2026-07-07 hardening).
                    newest_processed_ns_.store(max_time_ns, std::memory_order_relaxed);
                    dmsg.rt_factor = rt_factor_.load(std::memory_order_relaxed);
                    {
                        // Data-time backlog: newest ARRIVED stamp (per-lidar
                        // atomic, callback-written) minus newest PROCESSED
                        // time. Rate-independent overload ground truth.
                        int64_t newest_arrived = 0;
                        for (const auto& [bn, bd] : lidars_data) {
                            newest_arrived = std::max(
                                newest_arrived,
                                bd.last_t_ns.load(std::memory_order_relaxed));
                        }
                        dmsg.backlog_ms = (newest_arrived > max_time_ns)
                            ? static_cast<float>(newest_arrived - max_time_ns) / 1e6f
                            : 0.f;
                    }
                    // Wall latency: node clock vs newest processed stamp.
                    // Meaningful live or with bag --clock + use_sim_time.
                    dmsg.latency_wall_ms = static_cast<float>(
                        this->now().nanoseconds() - max_time_ns) / 1e6f;
                    dmsg.shed_scans = shed_scans_.load(std::memory_order_relaxed);
                    dmsg.cycle_overruns = cycle_overruns_.load(std::memory_order_relaxed);
                    dmsg.gap_extrap_knots = if_lidar_only
                        ? estimator_lo.gapExtrapKnots()
                        : estimator_lio.gapExtrapKnots();
                    // TF ownership guard.
                    dmsg.tf_foreign_same_pair = tf_own_monitor_.foreignSamePair();
                    dmsg.tf_foreign_other_parent = tf_own_monitor_.foreignOtherParent();
                    dmsg.tf_conflict_active = tf_own_monitor_.foreignActiveWithin(
                            this->now().nanoseconds(), tf_conflict_quiet_ns_)
                        || tf_own_monitor_.foreignStaticSeen();
                    dmsg.tf_yielding = tf_yield_active_.load(std::memory_order_relaxed);
                    // LiDAR row gating (2026-07-26 accuracy pass).
                    dmsg.cov_escape_admits = if_lidar_only
                        ? estimator_lo.covEscapeAdmits() : estimator_lio.covEscapeAdmits();
                    dmsg.lidar_gate_rejected = if_lidar_only
                        ? estimator_lo.mahalRejected() : estimator_lio.mahalRejected();
                    dmsg.lidar_gate_disarms = if_lidar_only
                        ? estimator_lo.mahalDisarms() : estimator_lio.mahalDisarms();
                    dmsg.cov_sanitized = cov_sanitized_.load(std::memory_order_relaxed);
                    pub_diag_->publish(dmsg);
                }

                // Update diagnostics at 1 Hz. llabs: the node clock can jump
                // backwards (bag loop restart under use_sim_time) — a plain
                // difference would silence the tick until the clock re-reaches
                // the stale stamp.
                if (std::llabs(this->now().nanoseconds() - last_process_ns_.load(std::memory_order_relaxed)) >= 1000000000LL) {
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
                // Overload metric: steady-state iterations exceeding the 50 ms
                // design budget (kRatePeriod). Idle passes take ~1 ms and
                // cannot trip this; a climbing counter = the machine cannot
                // keep up with the configured workload.
                if (std::chrono::steady_clock::now() - cycle_t0 >
                        std::chrono::milliseconds(50)) {
                    cycle_overruns_.fetch_add(1, std::memory_order_relaxed);
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
    // Phase 3.1: knots retained by the sliding-window prune each cycle.
    // 0 disables pruning (pre-3.1 unbounded-growth behaviour).
    int spline_prune_keep_knots_ = 600;
    
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
    // Publish batches whose pose/twist covariance failed the on-the-wire sanity
    // check (non-finite entry, or a non-positive diagonal). Should stay 0; any
    // non-zero value is a bug report. Written by the worker, read by
    // updateDiagnostics on the executor thread.
    std::atomic<uint64_t> cov_sanitized_{0};
    // Phase-0 hardening instrumentation: cached under spline_mutex_ by the
    // worker at the end of each IEKF cycle, read lock-free by updateDiagnostics
    // (which may run on the ROS executor thread via the Updater's internal
    // timer, not just from the worker's force_update). Atomic so the read is
    // not a data race; stale-by-one-cycle is acceptable.
    std::atomic<int64_t> cached_spline_knots_{0};
    // Total knots ever created (numKnots + pruned). With Phase 3.1 pruning
    // active, "Spline Knots" plateaus at the retention window; this one keeps
    // the original unbounded-growth signal visible in diagnostics.
    std::atomic<int64_t> cached_spline_total_knots_{0};
    // HARDENING §3.2 correspondence funnel (cumulative; worker adds each IEKF
    // update's CorrespStats, updateDiagnostics publishes per-window deltas).
    std::atomic<uint64_t> corresp_candidates_{0};
    std::atomic<uint64_t> corresp_passed_knn_{0};
    std::atomic<uint64_t> corresp_passed_plane_{0};
    std::atomic<uint64_t> corresp_used_{0};
    // Previous-window snapshots, touched only inside updateDiagnostics.
    uint64_t last_corresp_candidates_ = 0;
    uint64_t last_corresp_passed_knn_ = 0;
    uint64_t last_corresp_passed_plane_ = 0;
    uint64_t last_corresp_used_ = 0;
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
    // HARDENING §3.3 runtime health monitors (utils/filter_health.h).
    // nis_detector_ is fed by the worker thread only; imu_health_ is fed by
    // getImuCallback under m_buff (sensor callbacks are mutually exclusive).
    // The atomics export their verdicts to updateDiagnostics on the executor
    // thread, same pattern as the cached_* metrics above.
    resple::health::NisDivergenceDetector nis_detector_;
    // HARDENING §3.3 recovery policy. OFF = detection-only (log +
    // diagnostics, legacy); HOLD = gate odometry/TF publication while
    // DIVERGED (released on full OK); RESET = reinflate the IEKF covariance
    // to the recovery covariance nis_reset_cov on DIVERGED (bug A2: the old
    // "configure-time prior" target was tiny and deflated the filter). nis_hold_active_ is
    // worker-thread-only; the reset counter is atomic for diagnostics.
    enum class NisRecoveryMode { OFF, HOLD, RESET };
    NisRecoveryMode nis_recovery_mode_ = NisRecoveryMode::OFF;
    std::atomic<bool> nis_hold_active_{false};  // worker writes, diagnostics reads
    std::atomic<uint64_t> nis_resets_{0};
    resple::health::ImuHealthMonitor imu_health_;
    std::atomic<int> filter_health_state_{0};        // FilterState as int
    std::atomic<double> nis_window_mean_{0.0};
    std::atomic<uint32_t> imu_fault_bits_{0};
    std::atomic<double> imu_gyro_bias_norm_{0.0};
    // Per-window baseline for IEKF numerical-failure deltas (published as a
    // per-second rate, not a cumulative count).
    uint64_t last_numerical_failures_lo_ = 0;
    uint64_t last_numerical_failures_lio_ = 0;
    // Per-window baseline for the TF-guard foreign-observation delta
    // (updateDiagnostics only, executor thread).
    uint64_t last_tf_foreign_total_ = 0;
    
    // Lifecycle management
    std::atomic<bool> processing_active_;
    std::thread processing_thread_;
    // Set by the worker lambda as its final action; consumed by
    // joinProcessingThreadBounded to decide join-vs-detach without two
    // threads ever touching processing_thread_ concurrently.
    std::atomic<bool> processing_thread_exited_{false};
    
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
    // Generic "PointCloud2" lidar type: field-introspection ingestion that
    // accepts any PointCloud2 layout (see utils/point_cloud_adapter.h).
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_generic;
    // Built once at subscription time from the LidarConfig overrides; the
    // layout itself is still resolved per message (cheap string compares)
    // because a relaunched driver may legitimately change the field set.
    resple::pc2::AdapterConfig generic_adapter_cfg_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cur_scan;
    rclcpp_lifecycle::LifecyclePublisher<estimate_msgs::msg::Estimate>::SharedPtr pub_est;
    rclcpp_lifecycle::LifecyclePublisher<estimate_msgs::msg::Diagnostics>::SharedPtr pub_diag_;
    // Phase 4: duration of the last completed async map update (µs).
    std::atomic<int64_t> last_map_update_us_{0};
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr pub_start_time;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    // Latched base->sensor extrinsic frames in YAML-extrinsic mode
    // (publish_extrinsic_tf; see latchExtrinsicTf). Null unless enabled.
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
    // Serializes static_br_->sendTransform: the LiDAR latch runs on the sensor
    // callback group, the IMU latch on the worker (initialization()), and
    // StaticTransformBroadcaster mutates its transform vector without a lock.
    std::mutex static_br_mutex_;
    // One-shot guard so diagnostics_.add runs exactly once across lifecycle
    // re-cycles (see on_configure).
    bool diag_registered_ = false;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_imu_transform_ = false;
    geometry_msgs::msg::TransformStamped imu_to_baselink_;
    // YAML IMU extrinsic (q_ib/t_ib params, 2026-07-11), pre-inverted to the
    // base<-imu TransformStamped form transformImu consumes. Consulted only
    // in the YAML convention (tf_extrinsics=false); false = identity default
    // (legacy pass-through). Set once in readParameters.
    bool have_yaml_imu_extrinsic_ = false;
    geometry_msgs::msg::TransformStamped yaml_imu_tf_;
    // Extrinsic convention switch (findings #5/#22/#26): true (default) = TF
    // carries the mounting extrinsic, YAML q_lb/t_lb is an extra offset on
    // top; false = upstream/dataset mode, no TF consulted, YAML applied once.
    // Per-LiDAR transform state lives in LidarConfig (sensor cb group only).
    bool tf_extrinsics_ = true;
    double tf_wait_timeout_ = 10.0;
    // IMU frame_id captured from the first sample (under m_buff); used by
    // initialization() to resolve the mounting rotation for gravity (finding #9).
    std::string imu_frame_id_;
    // LO-without-IMU init timeout state (finding #10; worker thread only).
    // NODE-CLOCK window (2026-07-08 clock-domain audit).
    resple::timeutil::RosTimeWait lo_imu_wait_;
    double lo_imu_wait_timeout_ = 10.0;

    // Initial-attitude source (Design B). "gravity" (default) = accelerometer
    // only, self-contained, BLOCKS until a stationary window exists. "tf" =
    // base_link attitude from the TF tree (init_attitude_frame -> frame_id),
    // works while moving. "tf_gravity_check" = TF authoritative, accelerometer
    // cross-checks it when a stationary window is available (WARN on
    // disagreement — a wrong imu->base_link extrinsic or a non-level attitude
    // frame shows up here). The attitude frame MUST be gravity-aligned (Z up);
    // TF supplies roll/pitch and yaw only if init_yaw_from_tf (else yaw stays a
    // free gauge, preserving the "relative odometry source" contract).
    std::string init_attitude_source_ = "gravity";
    std::string init_attitude_frame_;          // gravity-aligned reference frame
    bool init_yaw_from_tf_ = false;
    double gravity_magnitude_ = 9.81;
    double init_attitude_consistency_deg_ = 3.0;
    // TF-wait deadline for the attitude lookup (mirrors the LO-IMU wait): after
    // this, a tf-mode init that has neither TF nor a gravity window proceeds
    // with identity rather than blocking forever. NODE-CLOCK window.
    resple::timeutil::RosTimeWait init_attitude_wait_;
    // Last TF-vs-gravity roll/pitch disagreement (deg) from tf_gravity_check;
    // -1 = not computed. Read by the diagnostics thread.
    std::atomic<double> init_attitude_delta_deg_{-1.0};

    bool publish_tf, invert_tf;
    // dliio lesson (2026-07-10): when the YAML q_lb/t_lb convention is in
    // effect, latch the frame_id -> <sensor frame> extrinsic once on
    // /tf_static so the TF tree is complete without an external
    // static_transform_publisher (scripts/run_replay.sh has leaked stale
    // ones before). Default false — a bag's own /tf_static may already
    // define these frames (see config_07052026.yaml's frame prefixing).
    bool publish_extrinsic_tf_ = false;
    std::string frame_id;
    std::string odom_id;

    // TF ownership guard (2026-07-07): watches the raw /tf (+ /tf_static)
    // stream for a FOREIGN publisher on the pair this node broadcasts
    // (post-inversion odom->frame_id). Self-vs-foreign is stamp-matched
    // (utils/tf_ownership.h); the monitor's mutex is a leaf lock. Action:
    //   warn  (default) - throttled ERROR + diagnostics, transforms unchanged
    //   yield           - keep publishing odometry topics but suspend OUR TF
    //                     broadcast while the foreign publisher was seen
    //                     within tf_conflict_quiet_sec (auto-demotion; stops
    //                     the flicker without operator intervention)
    // When odom/publish_tf=false (external owner expected, e.g. the
    // robot_localization odom EKF), the guard instead runs the ABSENCE check:
    // one-shot WARN if nobody publishes the pair within tf_absent_warn_sec.
    resple::tfown::TfOwnershipMonitor tf_own_monitor_;
    std::string tf_conflict_action_ = "warn";
    int64_t tf_conflict_quiet_ns_ = 5000000000LL;
    double tf_absent_warn_sec_ = 10.0;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_watch_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_static_watch_;
    std::atomic<bool> tf_absence_warned_{false};
    std::atomic<bool> tf_yield_active_{false};  // diagnostics: yield currently holding our TF
    // Absence-check window in NODE-CLOCK time (2026-07-08 clock-domain audit):
    // sim-time aware, survives the pre-/clock zero reading and loop restarts.
    // Written from on_activate/heartbeat (worker) via elapsedSec; the guard's
    // freshness 'now' likewise comes from the node clock so yield/resume is
    // invariant to bag playback rate.
    resple::timeutil::RosTimeWait tf_absence_wait_;
    // Worker-only snapshot of lastPairSeenNs for the outage re-arm.
    int64_t tf_last_pair_seen_ns_ = 0;

    // Raw /tf + /tf_static watcher (default callback group; O(frames) string
    // compares per message, no allocation). Runs on the executor; the monitor
    // synchronizes internally against the worker's notePublished. from_static
    // marks /tf_static deliveries — a foreign STATIC claim is sticky in the
    // monitor (it persists in consumers' tf2 buffers forever).
    void tfWatchCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool from_static)
    {
        const int64_t now_ns = this->now().nanoseconds();
        for (const auto& ts : msg->transforms) {
            const int64_t stamp_ns = rclcpp::Time(ts.header.stamp).nanoseconds();
            const auto v = tf_own_monitor_.classify(
                ts.header.frame_id, ts.child_frame_id, stamp_ns, now_ns, from_static);
            if (!publish_tf) continue;  // demoted: foreign owner is EXPECTED here
            if (v == resple::tfown::Verdict::FOREIGN_SAME_PAIR) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RESPLE] TF ownership conflict: another publisher is broadcasting "
                    "%s->%s (%lu foreign transforms so far). Two publishers on one TF "
                    "pair make the pose flicker/jump. Either set odom/publish_tf: false "
                    "here (RESPLE stays an odometry source for the external EKF) or "
                    "disable the other publisher.%s",
                    tf_own_monitor_.parent().c_str(), tf_own_monitor_.child().c_str(),
                    static_cast<unsigned long>(tf_own_monitor_.foreignSamePair()),
                    tf_conflict_action_ == "yield"
                        ? " tf_conflict_action=yield: suspending our broadcast while "
                          "the other publisher is active." : "");
            } else if (v == resple::tfown::Verdict::FOREIGN_OTHER_PARENT) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RESPLE] TF tree conflict: frame '%s' is also claimed by parent "
                    "'%s' while we publish %s->%s (%lu so far). A TF child has exactly "
                    "one parent - the tree is broken and lookups will flap between "
                    "parents. Fix the URDF/static publisher or our frame_id.",
                    tf_own_monitor_.child().c_str(), ts.header.frame_id.c_str(),
                    tf_own_monitor_.parent().c_str(), tf_own_monitor_.child().c_str(),
                    static_cast<unsigned long>(tf_own_monitor_.foreignOtherParent()));
            }
        }
    }

    std::map<std::string, LidarConfig> lidars;
    float ds_lm_voxel;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_body;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last_ds;
    pcl::PointCloud<pcl::PointXYZINormal> pc_world;
    int point_filter_num = 1;
    int64_t time_offset = 0;

    std::vector<BoxPointType> cub_needrm;
    BoxPointType LocalMap_Points;
    // HARDENING §3.4 radius pruning state (see pruneMapRadius). The double /
    // Vector3d / bool members are only touched inside the async map-update
    // task (single in flight); the op counter is atomic for updateDiagnostics.
    double map_prune_radius_ = 0.0;  // 0 disables (cube-only legacy behaviour)
    Eigen::Vector3d last_radius_prune_center_ = Eigen::Vector3d::Zero();
    bool radius_prune_initialized_ = false;
    std::atomic<uint64_t> radius_prune_ops_{0};
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> accum_nearest_points;
    // Per-frame parallel buffer for k-NN results (used to live in PointData::nearest_points).
    // Sized to pt_meas.size() before each IEKF call; results moved into accum_nearest_points after.
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> pt_neighbors_;
    // HARDENING §6.3 internal-map insertion lag (map_insert_lag_knots, default
    // 0 = upstream insert-at-edge). Body-frame points are staged here until
    // their timestamps fall behind the convergence horizon (the IEKF only
    // updates the last 4 RCPs, so knots >= ~4 behind the edge are final), then
    // deskewed with those final knot values and released into pc_world for the
    // async ikd-Tree insert. Keeps trailing-edge jitter out of the REFERENCE
    // map the IEKF matches against (cf. SLICT's marginalization-time map
    // admission, arXiv:2211.03900; retrospective map refinement,
    // arXiv:2503.21293). Worker-thread-only state — no extra locking.
    struct StagedMapPoint {
        int64_t time_ns;
        pcl::PointXYZINormal pt;  // body frame
        Eigen::Quaterniond q_bl;
        Eigen::Vector3d t_bl;
        Eigen::aligned_vector<pcl::PointXYZINormal> neighbors;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    Eigen::aligned_deque<StagedMapPoint> map_insert_staging_;
    Eigen::aligned_vector<pcl::PointXYZINormal> released_w_;  // per-cycle scratch
    int map_insert_lag_knots_ = 0;
    // §6.3 covariance gate (VoxelMap's uncertainty-convergence criterion in
    // cheap form, arXiv:2109.07082): when > 0, staged points are released
    // EARLY whenever the edge-pose orientation std (sqrt of the rotational
    // covariance trace via getLastPoseCovariance) is below this many degrees.
    // Gentle motion gets a fresh reference map; aggressive motion gets the
    // full fixed-knot hold. Only meaningful with map_insert_lag_knots > 0.
    double map_insert_cov_gate_deg_ = 0.0;
    // §6.3 knot under-resolution diagnostic (Coco-LIC / ATI-CTLO lesson:
    // fixed knot_hz under-fits violent rotation). Each knot's FINAL ort_delta
    // norm — the rotation the spline must absorb in one knot interval — is
    // checked once when the knot leaves the est window. Worker-only cursor;
    // atomics for the executor-thread diagnostics reader.
    double knot_rotation_warn_rad_ = 0.05;
    int64_t knot_rot_checked_ = 0;
    std::atomic<uint64_t> knot_rot_warn_count_{0};
    std::atomic<uint32_t> knot_rot_max_mrad_{0};
    // §6.3 / §3.2 LiDAR localizability diagnostic (X-ICP family, report-only).
    // Per update, the information matrices of the point-to-plane constraints:
    //   E_tt = (1/N) Σ n nᵀ        (translation, world-frame-invariant scale)
    //   E_rr = (1/N) Σ (p×n)(p×n)ᵀ (rotation, body-frame lever arms)
    // kept as SEPARATE 3×3 blocks — a joint 6×6 mixes meter-scaled rotation
    // rows with unit normals and its condition number becomes
    // scale-dependent (the arXiv:2408.11809 field-analysis pitfall). Min
    // eigenvalue ≈ how constrained the weakest axis is; condition number ≈
    // anisotropy. Report-only by design: hard eigenvalue gates are brittle
    // across environments (same field analysis); these fields exist to
    // decide §3.2 (plane_min_cond_ratio) from bag evidence.
    std::atomic<float> degen_min_eig_trans_{0.f};
    std::atomic<float> degen_min_eig_rot_{0.f};
    std::atomic<float> degen_cond_trans_{0.f};
    std::atomic<float> degen_cond_rot_{0.f};
    // §3.2 degeneracy-gate activity (0 axes = gate idle / disabled).
    std::atomic<int> loc_gate_axes_diag_{0};
    std::atomic<uint64_t> loc_gate_updates_diag_{0};
    // §6.7 advisory cov inflation: largest std-dev (m) currently added to the
    // published pose covariance along gated axes (0 when the feature is off).
    std::atomic<double> loc_gate_cov_infl_diag_{0.0};
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
    // HARDENING §2.3: bounded input buffers. 0 = unbounded (legacy); when
    // set, the scan push drops the OLDEST scan (pc_buff+t_buff in lock-step)
    // and the IMU staging cap below replaces the old hardcoded 2000. Drops
    // are counted for diagnostics — on a healthy system both stay 0.
    int max_scan_buffer_ = 0;
    int max_imu_staging_ = 2000;
    std::atomic<uint64_t> dropped_scans_{0};
    std::atomic<uint64_t> dropped_imu_{0};

    // 2026-07-07 overload hardening ("drop scans, stay real-time"):
    // per-LiDAR DATA-TIME latency budget for queued raw scans. 0 = off.
    // Shedding happens in pushScanBounded via overload::latencyShedCount;
    // counted separately from the count-cap drops above so operators can
    // tell "budget engaged" from "burst overflowed the cap".
    int64_t max_latency_ns_ = 0;
    std::atomic<uint64_t> shed_scans_{0};
    // Worker steady-state iterations that exceeded the 50 ms budget.
    std::atomic<uint32_t> cycle_overruns_{0};
    // Real-time factor over the ~2 s heartbeat window (worker-written,
    // diagnostics-read). rtf_prev_* are worker-thread-only.
    std::atomic<float> rt_factor_{0.f};
    int64_t rtf_prev_spline_max_ = 0;
    std::chrono::steady_clock::time_point rtf_prev_wall_{};
    // Newest processed measurement stamp (worker-written) for latency_wall_ms
    // and backlog_ms; and cumulative gap-extrapolated knots (see Estimator).
    std::atomic<int64_t> newest_processed_ns_{0};
    // Publisher relief (overload hardening): both default true (= today).
    // publish_current_scan gates the /current_scan world-cloud copy+publish
    // (visualization only). publish_est_window gates /est_window — ONLY
    // disable together with use_mapping:=false (it is Mapping's sole input).
    // Worker-thread reads; set once in readParameters (configure).
    bool publish_current_scan_ = true;
    bool publish_est_window_ = true;

    // Single push path for every LiDAR callback (HARDENING §2.3).
    template<typename PtsT>
    void pushScanBounded(LidarData& ld, PtsT&& pts, int64_t t_begin)
    {
        std::lock_guard<std::mutex> lock(ld.mtx_pc);
        if (max_scan_buffer_ > 0) {
            while (static_cast<int>(ld.pc_buff.size()) >= max_scan_buffer_) {
                ld.pc_buff.pop_front();
                ld.t_buff.pop_front();
                dropped_scans_.fetch_add(1, std::memory_order_relaxed);
            }
        }
        // 2026-07-07 overload hardening: latency-bounded shedding ("drop
        // scans, stay real-time"). DATA-TIME budget against THIS lidar's
        // incoming stamp — playback-rate independent, no wall clock, and a
        // lagging second lidar is judged only against itself. This is the
        // single choke point every LiDAR callback routes through, already
        // under mtx_pc, and shedding here (before removeNaN/voxel/PointData)
        // is the cheapest possible drop. It also bounds pc_buff growth while
        // the worker is stuck in a long catch-up pass. The estimator sees a
        // clean time gap and fast-forwards via propRCP (damped when
        // gap_extrap_decay < 1) instead of processing stale data late.
        if (max_latency_ns_ > 0) {
            const std::size_t n = resple::overload::latencyShedCount(
                ld.t_buff, t_begin, max_latency_ns_);
            for (std::size_t i = 0; i < n; ++i) {
                ld.pc_buff.pop_front();
                ld.t_buff.pop_front();
            }
            if (n > 0) {
                shed_scans_.fetch_add(n, std::memory_order_relaxed);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RESPLE] latency budget engaged: shed %zu queued scan(s) "
                    "older than %ld ms (cumulative %lu). The machine is not "
                    "keeping up with the input rate.",
                    n, static_cast<long>(max_latency_ns_ / 1000000),
                    static_cast<unsigned long>(
                        shed_scans_.load(std::memory_order_relaxed)));
            }
        }
        ld.pc_buff.push_back(std::forward<PtsT>(pts));
        ld.t_buff.push_back(t_begin);
    }

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
    // Per-knot-step bias random-walk variance (LIO state rows 24-29). Defaults
    // reproduce the pre-fix accidental Q_block_new magnitude — see initFilter.
    double cov_bias_acc_rw_ = 0.0;
    double cov_bias_gyro_rw_ = 0.0;
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
                // clear() first: flatten APPENDS, and PCL_Storage is a member
                // of the tree shared with the CUDA map-refresh path (which
                // does clear it — see the two sites near lines 1447 and 4717).
                // Without this the saved map carried whatever the previous
                // flatten left behind and grew on every SaveMap invocation.
                ikdtree.PCL_Storage.clear();
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
        // Dense publish floor (0 = off — the pre-feature behaviour: one pose
        // per knot in steady state, holes when the edge jumps several knots).
        // See publishPoseAndTf / utils/dense_pub.h.
        // Clamped to <= 1 kHz. The binding constraint is the worst-case
        // back-fill BURST vs the TF-guard self-stamp ring, not the steady
        // rate: one publishPoseAndTf call can emit up to
        // 1 kHz x kDensePubMaxBackfillNs = ~1001 stamps before the executor
        // drains a single DDS loopback echo, and every one must still be in
        // the ring when its echo classifies or the node flags ITSELF as a
        // foreign publisher (ERROR spam in 'warn'; a self-inflicted TF hole
        // in 'yield'). kRingSlots = 1024 covers it; the static_assert in
        // test_tf_ownership.cpp pins the invariant.
        static_assert(1001 <= static_cast<int64_t>(
                          resple::tfown::TfOwnershipMonitor::kRingSlots),
                      "dense back-fill worst burst must fit the TF-guard ring");
        const double dense_pub_hz = CommonUtils::readParam<double>(
            this->get_node_parameters_interface(), "odom/dense_pub_hz", 0.0);
        dense_pub_interval_ns_ = dense_pub_hz > 0.0
            ? std::max<int64_t>(static_cast<int64_t>(1e9 / dense_pub_hz), 1000000)
            : 0;
        if (dense_pub_interval_ns_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] dense odom/TF back-fill enabled: %.1f Hz (interval %.1f ms)",
                1e9 / static_cast<double>(dense_pub_interval_ns_),
                static_cast<double>(dense_pub_interval_ns_) / 1e6);
        }
        frame_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "frame_id", "base_link");
        odom_id = CommonUtils::readParam<std::string>(this->get_node_parameters_interface(), "odom/frame_id", "odom");

        RCLCPP_INFO(this->get_logger(), "Frame IDs - odom: %s, body: %s",
                    odom_id.c_str(), frame_id.c_str());

        // TF ownership guard (2026-07-07). Watched pair = the POST-inversion
        // pair actually broadcast. configure() also resets the monitor for
        // lifecycle re-cycles (readParameters runs from on_configure).
        tf_conflict_action_ = CommonUtils::readParam<std::string>(
            this->get_node_parameters_interface(), "tf_conflict_action", std::string("warn"));
        if (tf_conflict_action_ != "warn" && tf_conflict_action_ != "yield") {
            RCLCPP_WARN(this->get_logger(),
                "[RESPLE] tf_conflict_action='%s' unknown (warn|yield); using 'warn'",
                tf_conflict_action_.c_str());
            tf_conflict_action_ = "warn";
        }
        tf_conflict_quiet_ns_ = static_cast<int64_t>(1e9 * CommonUtils::readParam<double>(
            this->get_node_parameters_interface(), "tf_conflict_quiet_sec", 5.0));
        tf_absent_warn_sec_ = CommonUtils::readParam<double>(
            this->get_node_parameters_interface(), "tf_absent_warn_sec", 10.0);
        if (invert_tf) {
            tf_own_monitor_.configure(frame_id, odom_id);
        } else {
            tf_own_monitor_.configure(odom_id, frame_id);
        }
        tf_absence_warned_.store(false, std::memory_order_relaxed);
        tf_yield_active_.store(false, std::memory_order_relaxed);

        // Extrinsic convention (see updateLidarTransform): default true keeps
        // the production TF-based flow; dataset replay launches pass false so
        // the YAML q_lb/t_lb is the single source of truth (upstream parity).
        tf_extrinsics_ = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "tf_extrinsics", true);
        publish_extrinsic_tf_ = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "publish_extrinsic_tf", false);
        // YAML IMU extrinsic (2026-07-11): p_imu = q_ib * p_body + t_ib —
        // the same direction convention as the per-lidar q_lb/t_lb. Only
        // consulted in the YAML convention (tf_extrinsics=false); TF mode
        // gets the IMU mounting from the tree. Identity (default) keeps the
        // legacy pass-through bit-identical.
        {
            std::vector<double> q_ib_v = CommonUtils::readParam<std::vector<double>>(
                this->get_node_parameters_interface(), "q_ib", {1.0, 0.0, 0.0, 0.0});
            Eigen::Quaterniond q_ib(q_ib_v.at(0), q_ib_v.at(1), q_ib_v.at(2), q_ib_v.at(3));
            q_ib.normalize();
            const Eigen::Vector3d t_ib = CommonUtils::readVector3d(
                this->get_node_parameters_interface(), "t_ib");
            have_yaml_imu_extrinsic_ =
                q_ib.angularDistance(Eigen::Quaterniond::Identity()) > 1e-6 ||
                t_ib.norm() > 1e-6;
            if (have_yaml_imu_extrinsic_) {
                const Eigen::Quaterniond q_bi = q_ib.inverse();
                const Eigen::Vector3d t_bi = q_bi * (-t_ib);
                yaml_imu_tf_ = geometry_msgs::msg::TransformStamped();
                yaml_imu_tf_.transform.rotation.w = q_bi.w();
                yaml_imu_tf_.transform.rotation.x = q_bi.x();
                yaml_imu_tf_.transform.rotation.y = q_bi.y();
                yaml_imu_tf_.transform.rotation.z = q_bi.z();
                yaml_imu_tf_.transform.translation.x = t_bi.x();
                yaml_imu_tf_.transform.translation.y = t_bi.y();
                yaml_imu_tf_.transform.translation.z = t_bi.z();
                if (tf_extrinsics_) {
                    RCLCPP_WARN(this->get_logger(),
                        "[RESPLE] q_ib/t_ib set but tf_extrinsics=true: the YAML IMU "
                        "extrinsic is IGNORED in TF mode (the tree carries the "
                        "mounting). Set tf_extrinsics: false or remove q_ib/t_ib.");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "[RESPLE] YAML IMU extrinsic armed (q_ib/t_ib): IMU samples "
                        "and gravity init will be rotated into the body frame.");
                }
            }
        }
        tf_wait_timeout_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "tf_wait_timeout", 10.0);
        lo_imu_wait_timeout_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "lo_imu_wait_timeout", 10.0);

        // Initial-attitude source (Design B — see the member declarations).
        init_attitude_source_ = CommonUtils::readParam<std::string>(
            this->get_node_parameters_interface(), "init_attitude_source", std::string("gravity"));
        init_attitude_frame_ = CommonUtils::readParam<std::string>(
            this->get_node_parameters_interface(), "init_attitude_frame", std::string(""));
        init_yaw_from_tf_ = CommonUtils::readParam<bool>(
            this->get_node_parameters_interface(), "init_yaw_from_tf", false);
        gravity_magnitude_ = CommonUtils::readParam<double>(
            this->get_node_parameters_interface(), "gravity_magnitude", 9.81);
        init_attitude_consistency_deg_ = CommonUtils::readParam<double>(
            this->get_node_parameters_interface(), "init_attitude_consistency_deg", 3.0);
        const bool init_tf_mode = (init_attitude_source_ == "tf" ||
                                   init_attitude_source_ == "tf_gravity_check");
        if (init_tf_mode && init_attitude_frame_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "[RESPLE] init_attitude_source='%s' needs init_attitude_frame (a "
                "gravity-aligned frame); none set — falling back to gravity alignment.",
                init_attitude_source_.c_str());
            init_attitude_source_ = "gravity";
        } else if (init_tf_mode) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] initial attitude from TF '%s' -> '%s' (mode=%s, yaw %s, "
                "consistency gate %.1f deg)", init_attitude_frame_.c_str(), frame_id.c_str(),
                init_attitude_source_.c_str(), init_yaw_from_tf_ ? "from TF" : "free",
                init_attitude_consistency_deg_);
        }
        if (!tf_extrinsics_) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] tf_extrinsics=false: sensor clouds/IMU stay in their native "
                "frames; YAML q_lb/t_lb extrinsics applied once (upstream convention)");
        }
        
        // Voxel / gating parameters. These were "optional" with a 0.0 default,
        // but 0.0 is degenerate, not neutral: a zero VoxelGrid leaf makes PCL's
        // index math divide by zero, a zero ikd-Tree downsample box disables
        // map sparsification, and nn_thresh=0 rejects every point-to-plane
        // residual in the primary gate. Fall back to the canonical config
        // values with a loud warning instead (finding #27).
        ds_lm_voxel = CommonUtils::readParam<float>(this->get_node_parameters_interface(), "ds_lm_voxel", 0.0);
        if (ds_lm_voxel <= 0.0f) {
            RCLCPP_WARN(this->get_logger(),
                "ds_lm_voxel missing or non-positive (%.3f); using 0.2 m", ds_lm_voxel);
            ds_lm_voxel = 0.2f;
        }

        // Validate ds_scan_voxel parameter
        float ds_scan_voxel = CommonUtils::readParam<float>(this->get_node_parameters_interface(), "ds_scan_voxel", 0.0);
        if (ds_scan_voxel <= 0.0f) {
            RCLCPP_WARN(this->get_logger(),
                "ds_scan_voxel missing or non-positive (%.3f); using 0.2 m", ds_scan_voxel);
            ds_scan_voxel = 0.2f;
        } else if (ds_scan_voxel < 0.01 || ds_scan_voxel > 1.0) {
            RCLCPP_WARN(this->get_logger(),
                "ds_scan_voxel value %.3f outside recommended range [0.01, 1.0]", ds_scan_voxel);
        }
        ds_filter_body.setLeafSize(ds_scan_voxel, ds_scan_voxel, ds_scan_voxel);

        // Validate nn_thresh parameter
        param.nn_thresh = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "nn_thresh", 0.0);
        if (param.nn_thresh <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                "nn_thresh missing or non-positive (%.3f); using 0.5 m", param.nn_thresh);
            param.nn_thresh = 0.5;
        } else if (param.nn_thresh < 0.1 || param.nn_thresh > 5.0) {
            RCLCPP_WARN(this->get_logger(),
                "nn_thresh value %.3f outside recommended range [0.1, 5.0]", param.nn_thresh);
        }
        // HARDENING §3.2 correspondence thresholds. Defaults reproduce the
        // previously hardcoded values exactly; plane_min_cond_ratio > 0
        // additionally rejects degenerate (collinear / rank-deficient)
        // neighbor patches — off by default until tuned against a known-good
        // bag (see HARDENING.md §3.2 for the benchmark requirement).
        {
            Association::CorrespConfig ccfg;
            ccfg.max_neighbor_sq_dist = static_cast<float>(
                CommonUtils::readParam<double>(this->get_node_parameters_interface(), "nn_max_sq_dist", 5.0));
            ccfg.plane_fit_thresh = static_cast<float>(
                CommonUtils::readParam<double>(this->get_node_parameters_interface(), "plane_fit_thresh", 0.1));
            ccfg.plane_min_cond_ratio = static_cast<float>(
                CommonUtils::readParam<double>(this->get_node_parameters_interface(), "plane_min_cond_ratio", 0.0));
            if (ccfg.max_neighbor_sq_dist <= 0.0f || ccfg.plane_fit_thresh <= 0.0f) {
                RCLCPP_WARN(this->get_logger(),
                    "nn_max_sq_dist=%.3f / plane_fit_thresh=%.3f must be > 0; using defaults 5.0 / 0.1",
                    ccfg.max_neighbor_sq_dist, ccfg.plane_fit_thresh);
                ccfg = Association::CorrespConfig{};
            }
            estimator_lo.corresp_cfg = ccfg;
            estimator_lio.corresp_cfg = ccfg;
        }

        // HARDENING §3.2 robust kernel on the point-to-plane residuals
        // (M-estimator IRLS weight on each point's information; softens the
        // binary accept-reject cliff). "none" preserves legacy weighting
        // exactly; off by default pending bag A/B via the funnel +
        // localizability diagnostics.
        {
            const std::string kernel = CommonUtils::readParam<std::string>(
                this->get_node_parameters_interface(), "robust_kernel", std::string("none"));
            double kernel_delta = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "robust_kernel_delta", 0.1);
            int kernel_id = 0;
            if (kernel == "huber") kernel_id = 1;
            else if (kernel == "cauchy") kernel_id = 2;
            else if (kernel != "none") {
                RCLCPP_WARN(this->get_logger(),
                    "robust_kernel='%s' unknown (none|huber|cauchy); using none", kernel.c_str());
            }
            if (kernel_delta <= 0.0) {
                RCLCPP_WARN(this->get_logger(),
                    "robust_kernel_delta=%.3f must be > 0; using 0.1", kernel_delta);
                kernel_delta = 0.1;
            }
            estimator_lo.robust_kernel = kernel_id;
            estimator_lio.robust_kernel = kernel_id;
            estimator_lo.robust_delta = kernel_delta;
            estimator_lio.robust_delta = kernel_delta;
            if (kernel_id != 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] robust_kernel=%s (delta=%.3f m) on LiDAR residuals.",
                    kernel.c_str(), kernel_delta);
            }
        }

        // Close-range measurement down-weighting (commit 3bffada),
        // parameterized 2026-07-02: the fixed 3 m reference protects against
        // one nearby dominant surface in open scenes, but in a narrow tunnel
        // (every return < range_ref) it starves the whole update 9-36x —
        // observed in the field as jittery/"panicking" behaviour with many
        // close points. Lower range_ref (e.g. 1.0) or set 0 to disable for
        // narrow-space missions. Defaults reproduce the previous behaviour
        // exactly (cap 900 = the old implicit (3/0.1)^2 ceiling).
        {
            const double rref = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "range_ref", 3.0);
            const double rmax = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "range_noise_scale_max", 900.0);
            estimator_lo.range_ref = rref;
            estimator_lio.range_ref = rref;
            estimator_lo.range_noise_scale_max = rmax;
            estimator_lio.range_noise_scale_max = rmax;
            // Relative (per-scan) close-range down-weighting. The absolute
            // form is scene-blind: in a confined space (tunnel / culvert /
            // truck bed) EVERY return is inside range_ref, so every row is
            // inflated by the same 9-36x, which does not re-balance anything
            // — it just scales the whole LiDAR information block down against
            // the prior, so the filter coasts on the prior/IMU and then snaps
            // when geometry returns. Relative mode normalizes by the scan's
            // farthest valid return: unchanged in open scenes (the normalizer
            // is 1 there, so anti-domination is preserved exactly), no
            // starvation when everything is close. Default false = legacy,
            // bit-identical. See utils/range_weighting.h.
            const bool rrel = CommonUtils::readParam<bool>(
                this->get_node_parameters_interface(), "range_noise_relative", false);
            estimator_lo.range_noise_relative = rrel;
            estimator_lio.range_noise_relative = rrel;
            if (rrel && rref > 0.0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] range_noise_relative=true: close-range variance "
                    "inflation (range_ref=%.2f m) normalized per scan by the "
                    "farthest valid return — confined spaces keep full LiDAR "
                    "authority.", rref);
            }
        }

        // Adaptive (Mahalanobis) LiDAR outlier gate — utils/lidar_gate.h.
        //
        // Motivation: the shipped accept test is `|zp| < nn_thresh ||
        // lid_cov < var_pt*coeff_cov`, and with coeff_cov=3.0 the second
        // disjunct holds for ANY converged covariance, so it short-circuits and
        // nn_thresh never binds. findCorresp's `|pd2| < sqrt(range)/9` is then
        // the only outlier rejection that fires, and it admits 0.35 m of
        // off-plane error at 10 m / 0.79 m at 50 m at FULL weight (the robust
        // kernel is off by default). This gate normalizes the residual by the
        // predicted innovation variance instead, so it relaxes while the filter
        // is uncertain (init, post-gap) and tightens once converged — and it
        // disarms itself when it would reject too much, which is the signature
        // of a wrong prior rather than bad data.
        //
        // Default sigma 0 = off, per the HARDENING category-8 rule (outlier
        // rejection changes wait on bag A/B). The cov_escape_admits counter
        // below ships regardless, so the exposure is measurable BEFORE flipping
        // this on: it counts rows admitted only by the cov_thresh disjunct.
        {
            const double gsigma = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "lidar_gate_sigma", 0.0);
            double gfrac = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "lidar_gate_max_reject_frac", 0.5);
            if (gfrac < 0.0 || gfrac > 1.0) {
                RCLCPP_WARN(this->get_logger(),
                    "lidar_gate_max_reject_frac=%.3f outside [0,1]; using 0.5", gfrac);
                gfrac = 0.5;
            }
            estimator_lo.lidar_gate.sigma = gsigma;
            estimator_lio.lidar_gate.sigma = gsigma;
            estimator_lo.lidar_gate.max_reject_frac = gfrac;
            estimator_lio.lidar_gate.max_reject_frac = gfrac;
            if (gsigma > 0.0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] lidar_gate_sigma=%.2f (disarm above %.0f%% rejected): "
                    "point-to-plane rows beyond %.2f sigma of the predicted "
                    "innovation are dropped. With var_pt=0.01 that is ~%.2f m "
                    "once converged, widening while the covariance is large.",
                    gsigma, gfrac * 100.0, gsigma, gsigma * 0.1);
            }
        }

        // Overload hardening (2026-07-07): damped propRCP gap extrapolation.
        // decay 1.0 (default) is a bit-identical no-op — propRCP takes the
        // literal legacy constant-velocity expression. decay < 1 geometrically
        // damps the extrapolated velocity for knots beyond free_knots within
        // one propRCP call, bounding the excursion (and the post-gap yank)
        // after latency sheds / scan drops. See utils/overload_control.h.
        {
            const double gap_decay = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "gap_extrap_decay", 1.0);
            const int gap_free = CommonUtils::readParam<int>(
                this->get_node_parameters_interface(), "gap_extrap_free_knots", 3);
            resple::overload::GapExtrapDamping gd;
            // Clamp out a negative decay: scale(k)=pow(decay,excess) sign-flips
            // for odd excess when decay<0, which would push propRCP's gap
            // extrapolation the WRONG direction. 0 = hard hold (the intended
            // floor); >=1 = off.
            if (gap_decay < 0.0) {
                RCLCPP_WARN(this->get_logger(),
                    "[RESPLE] gap_extrap_decay=%.3f < 0 is invalid; clamping to 0 (hard hold)",
                    gap_decay);
            }
            gd.decay = std::max(0.0, gap_decay);
            gd.free_knots = std::max(0, gap_free);
            estimator_lo.gap_extrap = gd;
            estimator_lio.gap_extrap = gd;
            if (gap_decay < 1.0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] gap_extrap_decay=%.3f (free_knots=%d): propRCP "
                    "extrapolation velocity damped geometrically beyond the free "
                    "window; excursion across a v m/knot gap bounded by ~free + "
                    "v/(1-decay) knots instead of growing linearly.",
                    gap_decay, gd.free_knots);
            }
        }

        // §3.2 prototype: X-ICP-style degenerate-direction gate (translation
        // only). 0 disables (default — decision-gate rule: this changes the
        // estimator's update; enable only behind a bag A/B). Recommended
        // trial gate 0.02 (E_tt eigenvalues sum to 1; healthy geometry runs
        // ~0.08+ min-eig, tunnel collapse 1e-3 and below).
        {
            const double loc_gate = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "loc_gate_trans_min_eig", 0.0);
            estimator_lo.loc_gate_trans_min_eig = loc_gate;
            estimator_lio.loc_gate_trans_min_eig = loc_gate;
            if (loc_gate > 0.0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] loc_gate_trans_min_eig=%.4f: LiDAR gain projected out of "
                    "translation directions whose constraint eigenvalue falls below the "
                    "gate; IMU rows and the spline prior keep full authority there "
                    "(degeneracy gate, HARDENING §3.2).", loc_gate);
            }
            // Publish-side advisory covariance inflation along persistently-gated
            // axes (HARDENING §6.7). Grows the REPORTED /odom translation
            // covariance toward realistic absolute uncertainty in a tunnel; the
            // internal filter and trajectory are untouched. 0 = off (default).
            const double cov_rate = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "loc_gate_cov_rate", 0.0);
            const double cov_decay = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "loc_gate_cov_decay", 0.99);
            const int cov_persist = CommonUtils::readParam<int>(
                this->get_node_parameters_interface(), "loc_gate_cov_persist", 5);
            const double cov_min_eig = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "loc_gate_cov_min_eig", 0.0);
            estimator_lo.loc_gate_cov_rate = cov_rate;
            estimator_lio.loc_gate_cov_rate = cov_rate;
            estimator_lo.loc_gate_cov_decay = cov_decay;
            estimator_lio.loc_gate_cov_decay = cov_decay;
            estimator_lo.loc_gate_cov_persist = cov_persist;
            estimator_lio.loc_gate_cov_persist = cov_persist;
            estimator_lo.loc_gate_cov_min_eig = cov_min_eig;
            estimator_lio.loc_gate_cov_min_eig = cov_min_eig;
            if (cov_rate > 0.0) {
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] loc_gate_cov_rate=%.4g (decay %.3f, persist %d): published "
                    "pose covariance inflated along gated axes after %d consecutive "
                    "gated frames; steady-state along-axis var ~%.3g m² (advisory, "
                    "filter untouched).", cov_rate, cov_decay, cov_persist, cov_persist,
                    cov_decay < 1.0 ? cov_rate / (1.0 - cov_decay) : 0.0);
            }
        }

        // HARDENING §3.3 (bug A2): covariance the 'reset' recovery reinflates to
        // on a DIVERGED verdict. NIS divergence signals an OVER-confident
        // (too-small) covariance, so the recovery target must be LARGE; the old
        // code reset to the tiny startup prior (cov_P0 ~ 2e-6), which deflated it
        // further and froze the filter. Default 1.0 (sigma ~1 m / 1 rad). Set on
        // both estimators so the mode works in LO or LIO; only consulted when
        // nis_recovery_mode=reset fires. setState() does not touch it.
        {
            const double nis_reset_cov = CommonUtils::readParam<double>(
                this->get_node_parameters_interface(), "nis_reset_cov", 1.0);
            estimator_lo.setRecoveryCovariance(nis_reset_cov);
            estimator_lio.setRecoveryCovariance(nis_reset_cov);
        }

        // HARDENING §3.1 sliding-window knot pruning. Default 600 knots
        // (6 s at the canonical knot_hz=100) bounds spline memory while
        // staying far wider than any backward-looking consumer. 0 disables.
        // Values below 100 keep too little margin for the startup gates
        // (collectMeasurements treats numKnots()<10 specially) and for
        // deskew under scan-buffer backlog, so clamp them up.
        spline_prune_keep_knots_ = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "spline_prune_keep_knots", 600);
        if (spline_prune_keep_knots_ != 0 && spline_prune_keep_knots_ < 100) {
            RCLCPP_WARN(this->get_logger(),
                "spline_prune_keep_knots=%d is below the safe minimum; using 100",
                spline_prune_keep_knots_);
            spline_prune_keep_knots_ = 100;
        }
        // HARDENING §2.3 bounded input buffers. max_scan_buffer caps each
        // per-LiDAR raw-scan deque in SCANS (0 = unbounded legacy);
        // max_imu_staging caps imu_int_buff in SAMPLES (default keeps the
        // previously hardcoded 2000; 0 disables). Drop-oldest + counters.
        max_scan_buffer_ = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "max_scan_buffer", 0);
        max_imu_staging_ = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "max_imu_staging", 2000);
        // 2026-07-07 overload hardening: per-LiDAR data-time latency budget.
        // 0 = off (default, legacy). See pushScanBounded / doc/PARAMETERS.md.
        max_latency_ns_ = 1000000LL * CommonUtils::readParam<int>(
            this->get_node_parameters_interface(), "max_latency_ms", 0);
        if (max_latency_ns_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] latency shedding armed: queued scans older than %ld ms "
                "(data time, per lidar) are dropped to stay real-time",
                static_cast<long>(max_latency_ns_ / 1000000));
        }
        // Publisher relief (overload hardening): both true by default.
        publish_current_scan_ = CommonUtils::readParam<bool>(
            this->get_node_parameters_interface(), "publish_current_scan", true);
        publish_est_window_ = CommonUtils::readParam<bool>(
            this->get_node_parameters_interface(), "publish_est_window", true);
        if (!publish_current_scan_) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] publish_current_scan=false: /current_scan disabled "
                "(visualization only; odometry/map unaffected).");
        }
        if (!publish_est_window_) {
            RCLCPP_WARN(this->get_logger(),
                "[RESPLE] publish_est_window=false: /est_window disabled. The "
                "Mapping node consumes ONLY this topic — run with "
                "use_mapping:=false or the map will silently never build.");
        }

        // HARDENING §6.3 internal-map insertion lag (0 = upstream
        // insert-at-edge; recommended trial value 8 = the convergence
        // horizon + margin). Delays ikd-Tree insertion — and therefore
        // /current_scan, which publishes the same released points — by
        // lag × dt so the REFERENCE map is built from final knot values.
        // Off by default pending bag validation (repo decision-gate rule).
        map_insert_lag_knots_ = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "map_insert_lag_knots", 0);
        if (map_insert_lag_knots_ < 0) map_insert_lag_knots_ = 0;
        if (map_insert_lag_knots_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] map_insert_lag_knots=%d: ikd-Tree insertion + /current_scan "
                "lag the spline edge by %d knots (odometry unaffected).",
                map_insert_lag_knots_, map_insert_lag_knots_);
        }
        // §6.3 covariance gate for early release of staged map points
        // (degrees of edge-pose orientation std; 0 = pure fixed-knot lag).
        map_insert_cov_gate_deg_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "map_insert_cov_gate_deg", 0.0);
        if (map_insert_cov_gate_deg_ < 0.0) map_insert_cov_gate_deg_ = 0.0;
        // §6.3 knot under-resolution warning threshold (rad of rotation per
        // knot interval; 0 disables). 0.05 rad/knot = 5 rad/s at knot_hz 100 —
        // well above any rover/handheld motion, fires on HelmDyn-class spin.
        knot_rotation_warn_rad_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "knot_rotation_warn_rad", 0.05);

        // HARDENING §3.3 divergence recovery policy: off (detection-only,
        // legacy) | hold (gate odometry/TF while DIVERGED) | reset
        // (reinflate IEKF covariance to the prior on DIVERGED).
        {
            const std::string mode = CommonUtils::readParam<std::string>(
                this->get_node_parameters_interface(), "nis_recovery_mode", std::string("off"));
            if (mode == "hold") {
                nis_recovery_mode_ = NisRecoveryMode::HOLD;
            } else if (mode == "reset") {
                nis_recovery_mode_ = NisRecoveryMode::RESET;
            } else {
                if (mode != "off") {
                    RCLCPP_WARN(this->get_logger(),
                        "nis_recovery_mode='%s' not recognized (off|hold|reset); using 'off'",
                        mode.c_str());
                }
                nis_recovery_mode_ = NisRecoveryMode::OFF;
            }
        }
        if_lidar_only = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "if_lidar_only", false);
        if (!if_lidar_only) {
            acc_ratio = CommonUtils::readParam<bool>(this->get_node_parameters_interface(), "acc_ratio", false);

            // HARDENING §3.3 divergence-detector tuning. Defaults suit the
            // 20 Hz worker; with nis_window=32 a verdict lands every ~1.6 s
            // and nis_breach_limit=3 means ~5 s of sustained inconsistency
            // before DIVERGED is declared.
            {
                resple::health::NisDetectorConfig nis_cfg;
                nis_cfg.window = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "nis_window", 32);
                nis_cfg.warn_ratio = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "nis_warn_ratio", 2.0);
                nis_cfg.diverged_ratio = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "nis_diverged_ratio", 4.0);
                nis_cfg.breach_limit = CommonUtils::readParam<int>(this->get_node_parameters_interface(), "nis_breach_limit", 3);
                nis_detector_ = resple::health::NisDivergenceDetector(nis_cfg);
                imu_health_ = resple::health::ImuHealthMonitor(resple::health::ImuHealthConfig{});
            }
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
        // Bias random-walk process noise (finding #8): defaults keep the
        // magnitude the bias block accidentally received before the fix.
        cov_bias_acc_rw_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(),
            "cov_bias_acc_rw", cov_RCP_pos_new * cov_sys_pos);
        cov_bias_gyro_rw_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(),
            "cov_bias_gyro_rw", cov_RCP_ort_new * cov_sys_ort);
        param.coeff_cov = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "coeff_cov", 10);

        cube_len = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "cube_len", 1000.0);
        // HARDENING §3.4: keep only map points within this distance of the
        // current pose (0 = disabled, legacy cube-only behaviour). Floored at
        // 2x det_range in pruneMapRadius; warn here so the operator sees the
        // effective value.
        map_prune_radius_ = CommonUtils::readParam<double>(this->get_node_parameters_interface(), "map_prune_radius", 0.0);
        if (map_prune_radius_ > 0.0 && map_prune_radius_ < 2.0 * double(det_range)) {
            RCLCPP_WARN(this->get_logger(),
                "map_prune_radius=%.1f is below 2x det_range (%.1f); the effective radius will be %.1f",
                map_prune_radius_, 2.0 * double(det_range), 2.0 * double(det_range));
        }
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
        // Newest-RCP block. Upstream wrote this via bottomRightCorner<6,6>()
        // on the 30x30 Q, which lands on the BIAS block (24-29): the newest
        // knot got ZERO process noise, and in LO mode Q_block_new was
        // discarded entirely by the topLeftCorner<24,24> slice (bug-hunt
        // 2026-07-02 finding #8; inherited from upstream).
        Q.block<6, 6>(18, 18) = Q_block_new;
        // Bias random walk (LIO state 24-29). Pre-fix, these rows received
        // Q_block_new by the indexing accident above; the defaults preserve
        // that magnitude so LIO bias dynamics are unchanged unless retuned.
        Q.block<3, 3>(24, 24) = cov_bias_acc_rw_ * Eigen::Matrix3d::Identity();
        Q.block<3, 3>(27, 27) = cov_bias_gyro_rw_ * Eigen::Matrix3d::Identity();
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

    // Best-effort one-shot lookup of base_link's attitude in the configured
    // gravity-aligned reference frame (Design B). Returns the rotation
    // R_{init_attitude_frame <- frame_id} at the latest available time. No
    // internal blocking — initialization() retries each cycle and bounds the
    // wait via init_attitude_wait_. Time(0) (latest) is used for the
    // same reason updateImuTransform/updateLidarTransform do: exact for a
    // static mount, close enough for a slowly-varying source at init.
    bool lookupInitAttitude(Eigen::Quaterniond& q_wb_out)
    {
        if (init_attitude_frame_.empty()) return false;
        try {
            if (!tf_buffer_->_frameExists(init_attitude_frame_) ||
                !tf_buffer_->_frameExists(this->frame_id)) {
                return false;
            }
            if (!tf_buffer_->canTransform(init_attitude_frame_, this->frame_id,
                                          rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0))) {
                return false;
            }
            const auto tf = tf_buffer_->lookupTransform(
                init_attitude_frame_, this->frame_id, rclcpp::Time(0));
            q_wb_out = Eigen::Quaterniond(tf2::transformToEigen(tf).rotation());
            q_wb_out.normalize();
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] init attitude TF lookup (%s -> %s) failed: %s",
                init_attitude_frame_.c_str(), this->frame_id.c_str(), ex.what());
            return false;
        }
    }

    bool updateImuTransform(std::string source_frame_id)
    {
        // YAML-only mode (tf_extrinsics=false): never consult TF. If a YAML
        // IMU extrinsic (q_ib/t_ib) is configured, apply IT through the same
        // transformImu path the TF mode uses (2026-07-11: closes the gap
        // where a tilted IMU was silently fused unrotated in YAML mode —
        // rotation, lever arm and the finding-#9 gravity-init rotation all
        // reuse the TF-mode machinery). Identity default = the legacy
        // pass-through, bit-identical.
        if (!tf_extrinsics_) {
            if (!have_yaml_imu_extrinsic_) {
                return false;
            }
            if (!have_imu_transform_) {
                imu_to_baselink_ = yaml_imu_tf_;
                imu_to_baselink_.header.frame_id = this->frame_id;
                imu_to_baselink_.child_frame_id = source_frame_id;
                have_imu_transform_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] IMU extrinsic from YAML q_ib/t_ib (%s -> %s): "
                    "t=[%.3f %.3f %.3f]",
                    this->frame_id.c_str(), source_frame_id.c_str(),
                    imu_to_baselink_.transform.translation.x,
                    imu_to_baselink_.transform.translation.y,
                    imu_to_baselink_.transform.translation.z);
                latchImuExtrinsicTf(source_frame_id);
            }
            return true;
        }
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
                // Zero timeout (2026-07-07 review): this runs on the sensor
                // callback thread HOLDING m_buff — a blocking wait here stalls
                // the worker (init/collectMeasurements) and, because the sensor
                // group is mutually exclusive, every other sensor callback.
                // Callers retry at message rate, so waiting buys nothing.
                if (tf_buffer_->canTransform(this->frame_id, source_frame_id,
                                                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0))) {
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

    // Per-LiDAR extrinsic resolution (bug-hunt 2026-07-02, findings #5/#22/#26).
    //
    // Extrinsic contract (documented in config_06042026.yaml): when
    // tf_extrinsics=true (default) the base_link<-sensor TF carries the
    // mounting extrinsic, the callback bakes it into the cloud, and the YAML
    // q_lb/t_lb is an EXTRA offset applied on top (identity in production).
    // When tf_extrinsics=false (upstream / dataset-replay mode) no TF is
    // consulted: clouds stay in the raw sensor frame and the YAML q_lb/t_lb
    // is applied exactly once per point, as upstream RESPLE did.
    //
    // Previous defects fixed here:
    //  * scans were dropped FOREVER when no TF was published (NTU/KTH replay
    //    launches) — now a timed fallback (tf_wait_timeout, default 10 s)
    //    degrades to the YAML-only convention with a one-shot warning;
    //  * a single lidar_to_baselink_ cache was shared across all LiDARs, so
    //    in multi-LiDAR configs every sensor got whichever frame latched
    //    first — the cache is now per-LidarConfig;
    //  * publishing a TF that duplicates a non-identity YAML extrinsic makes
    //    the two cancel (q_bl∘q_lb = I) — now detected and warned.
    bool updateLidarTransform(const std::string& source_frame_id, LidarConfig& lidar)
    {
        if (lidar.extrinsic_ready) {
            return true;
        }
        if (!tf_extrinsics_) {
            // YAML-only mode: identity TF, raw clouds, q_bl/t_bl applied once
            // in pointBodyToWorld / prepLiDAR. sensor_origin_body stays zero
            // (points remain in the sensor frame).
            lidar.extrinsic_ready = true;
            latchExtrinsicTf(source_frame_id, lidar);
            return true;
        }
        try {
            // Pre-check avoids tf2's "frame does not exist" warning while the
            // static TF publisher is still coming up.
            if (tf_buffer_->_frameExists(this->frame_id) &&
                tf_buffer_->_frameExists(source_frame_id) &&
                tf_buffer_->canTransform(this->frame_id, source_frame_id,
                                         rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0))) {
                const auto transform = tf_buffer_->lookupTransform(
                    this->frame_id, source_frame_id, rclcpp::Time(0));
                lidar.lidar_to_baselink = tf2::transformToEigen(transform);
                lidar.tf_latched = true;
                lidar.extrinsic_ready = true;
                // True sensor origin in body frame for the range-based
                // outlier gate — per-lidar, from THIS frame's TF.
                lidar.sensor_origin_body = lidar.lidar_to_baselink.translation();
                RCLCPP_INFO(this->get_logger(), "[RESPLE] Got LiDAR transform: %s -> %s",
                        source_frame_id.c_str(), this->frame_id.c_str());
                // Tripwire: TF and YAML both non-identity means the extrinsic
                // is applied twice (q_bl on an already-transformed cloud). If
                // the TF was copied from q_lb/t_lb verbatim the two cancel and
                // the extrinsic is silently NULLED (see doc/PARAMETERS.md).
                const bool yaml_nontrivial =
                    lidar.q_bl.angularDistance(Eigen::Quaterniond::Identity()) > 1e-3 ||
                    lidar.t_bl.norm() > 1e-3;
                const bool tf_nontrivial =
                    Eigen::Quaterniond(lidar.lidar_to_baselink.rotation())
                        .angularDistance(Eigen::Quaterniond::Identity()) > 1e-3 ||
                    lidar.lidar_to_baselink.translation().norm() > 1e-3;
                if (yaml_nontrivial && tf_nontrivial) {
                    RCLCPP_WARN(this->get_logger(),
                        "[RESPLE] LiDAR '%s': BOTH the TF (%s->%s) and the YAML q_lb/t_lb "
                        "extrinsics are non-identity. The YAML extrinsic is applied ON TOP "
                        "of the TF-transformed cloud — if both encode the same mounting "
                        "transform they cancel out. Set q_lb/t_lb to identity (TF carries "
                        "the extrinsic) or run with tf_extrinsics:=false (YAML carries it).",
                        source_frame_id.c_str(), source_frame_id.c_str(), this->frame_id.c_str());
                }
                return true;
            }
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "[RESPLE] LiDAR transform exception: %s", ex.what());
        }
        // TF not available yet: wait up to tf_wait_timeout_ seconds (measured
        // from this lidar's first attempt), then fall back to the YAML-only
        // convention instead of dropping scans forever. NODE-CLOCK time
        // (2026-07-08 clock-domain audit): with use_sim_time this window runs
        // in bag time, so throttled replay cannot expire it before the bag
        // publishes the TF; RosTimeWait also survives the pre-/clock zero
        // reading and bag-loop restarts.
        if (lidar.tf_wait.elapsedSec(this->now().nanoseconds()) > tf_wait_timeout_) {
            RCLCPP_WARN(this->get_logger(),
                "[RESPLE] No TF %s -> %s after %.1f s; falling back to the YAML "
                "q_lb/t_lb extrinsic only (upstream convention). Publish the TF or "
                "set tf_extrinsics:=false to silence this.",
                source_frame_id.c_str(), this->frame_id.c_str(), tf_wait_timeout_);
            lidar.extrinsic_ready = true;   // identity TF from here on
            latchExtrinsicTf(source_frame_id, lidar);
            return true;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "[RESPLE] Waiting for LiDAR transform: %s -> %s",
                            source_frame_id.c_str(), this->frame_id.c_str());
        return false;
    }

    // dliio lesson (2026-07-10 review): dliio broadcasts its param extrinsics
    // as base->imu/lidar TFs so its tree is complete out of the box; RESPLE in
    // the YAML q_lb/t_lb convention leaves the sensor frames dangling, forcing
    // replay harnesses to run an external static_transform_publisher (which
    // has leaked stale extrinsics across runs before — scripts/run_replay.sh).
    // When publish_extrinsic_tf is set, latch frame_id -> <cloud header frame>
    // ONCE on /tf_static with (q_bl, t_bl) — exactly the T_base<-sensor that
    // pointBodyToWorld applies, so TF consumers and the estimator agree by
    // construction. Latched (not dliio's per-cycle /tf re-broadcast): the
    // extrinsic is static, and /tf traffic stays untouched.
    //
    // LiDAR frames only: RESPLE has no YAML IMU extrinsic, and asserting an
    // identity base->imu we don't actually know would mislead downstream
    // (dliio's 06042026 180-yaw-as-identity bug is the cautionary tale).
    //
    // Runs on the sensor callback group (mutually exclusive) via
    // updateLidarTransform, once per lidar (extrinsic_ready gates re-entry).
    void latchExtrinsicTf(const std::string& source_frame_id, const LidarConfig& lidar)
    {
        if (!publish_extrinsic_tf_ || !static_br_) {
            return;
        }
        if (source_frame_id.empty() || source_frame_id == this->frame_id) {
            RCLCPP_WARN(this->get_logger(),
                "[RESPLE] publish_extrinsic_tf: cloud frame_id '%s' is empty or "
                "equals frame_id '%s' — no extrinsic TF latched for this LiDAR.",
                source_frame_id.c_str(), this->frame_id.c_str());
            return;
        }
        // Collision guard: if ANY transform already references this frame
        // (a bag's /tf_static, a URDF publisher), someone else owns it — a
        // second static latch on the same child would fight it (tf2 keeps
        // one parent per child). Best-effort: a late-arriving latched TF can
        // still slip past; prefix RESPLE's frames (config_07052026.yaml
        // pattern) when replaying a bag that carries its own tree.
        if (tf_buffer_ && tf_buffer_->_frameExists(source_frame_id)) {
            RCLCPP_WARN(this->get_logger(),
                "[RESPLE] publish_extrinsic_tf: frame '%s' already exists in the "
                "TF tree (bag /tf_static? URDF?) — not latching %s -> %s to avoid "
                "a two-owner conflict.",
                source_frame_id.c_str(), this->frame_id.c_str(), source_frame_id.c_str());
            return;
        }
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = this->frame_id;
        ts.child_frame_id = source_frame_id;
        ts.transform.translation.x = lidar.t_bl.x();
        ts.transform.translation.y = lidar.t_bl.y();
        ts.transform.translation.z = lidar.t_bl.z();
        ts.transform.rotation.w = lidar.q_bl.w();
        ts.transform.rotation.x = lidar.q_bl.x();
        ts.transform.rotation.y = lidar.q_bl.y();
        ts.transform.rotation.z = lidar.q_bl.z();
        {
            // static_br_ (tf2_ros::StaticTransformBroadcaster) mutates a shared
            // std::vector with no internal lock. This latch runs on the sensor
            // callback group; latchImuExtrinsicTf runs on the WORKER during
            // initialization() — a concurrent sendTransform on the same
            // broadcaster races the vector. Serialize with a leaf mutex.
            std::lock_guard<std::mutex> lk(static_br_mutex_);
            static_br_->sendTransform(ts);
        }
        RCLCPP_INFO(this->get_logger(),
            "[RESPLE] latched static extrinsic TF %s -> %s from YAML q_lb/t_lb "
            "(t=[%.3f %.3f %.3f])",
            this->frame_id.c_str(), source_frame_id.c_str(),
            lidar.t_bl.x(), lidar.t_bl.y(), lidar.t_bl.z());
    }

    // publish_extrinsic_tf companion for the IMU (2026-07-11): only fires
    // when a YAML q_ib/t_ib is actually configured — unlike the earlier
    // "asserting identity would be a guess" objection, here the mounting IS
    // known. Same collision guard as the LiDAR latch. Called once from
    // updateImuTransform's YAML path (the /tf_static publish is a
    // non-blocking DDS write; one-shot).
    void latchImuExtrinsicTf(const std::string& source_frame_id)
    {
        if (!publish_extrinsic_tf_ || !static_br_ || !have_yaml_imu_extrinsic_) {
            return;
        }
        if (source_frame_id.empty() || source_frame_id == this->frame_id) {
            return;
        }
        if (tf_buffer_ && tf_buffer_->_frameExists(source_frame_id)) {
            RCLCPP_WARN(this->get_logger(),
                "[RESPLE] publish_extrinsic_tf: IMU frame '%s' already exists in "
                "the TF tree - not latching %s -> %s to avoid a two-owner conflict.",
                source_frame_id.c_str(), this->frame_id.c_str(), source_frame_id.c_str());
            return;
        }
        geometry_msgs::msg::TransformStamped ts = yaml_imu_tf_;
        ts.header.stamp = this->now();
        ts.header.frame_id = this->frame_id;
        ts.child_frame_id = source_frame_id;
        {
            // See latchExtrinsicTf: this runs on the worker (initialization()),
            // the LiDAR latch on the sensor callback group — same lock-free
            // static_br_ vector. Serialize with the leaf mutex.
            std::lock_guard<std::mutex> lk(static_br_mutex_);
            static_br_->sendTransform(ts);
        }
        RCLCPP_INFO(this->get_logger(),
            "[RESPLE] latched static IMU extrinsic TF %s -> %s from YAML q_ib/t_ib",
            this->frame_id.c_str(), source_frame_id.c_str());
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

        // Transform linear acceleration, referring it from the IMU location to
        // the base_link origin (the spline/estimator reference point).
        // Rigid-body relation: a_origin = a_imu - alpha x r - omega x (omega x r),
        // where r = base_link origin -> IMU sensor (transform translation). The
        // centripetal term must be SUBTRACTED to remove it (bug A4: it was added,
        // which doubled the term instead of cancelling it). alpha x r (angular
        // acceleration) is not measured, so it is omitted.
        //
        // Units: omega x (omega x r) is in m/s^2, but with acc_ratio=true the
        // message accel is in g and stays in g until the worker scales the
        // whole vector by 9.81 on consumption. Express the correction in the
        // message's unit here so that later scaling does not multiply an
        // already-metric term by 9.81 (bug-hunt 2026-07-02 finding #7).
        Eigen::Vector3d lin_accel(imu_raw->linear_acceleration.x,
                                  imu_raw->linear_acceleration.y,
                                  imu_raw->linear_acceleration.z);
        const double accel_unit = acc_ratio ? (1.0 / 9.81) : 1.0;
        Eigen::Vector3d lin_accel_transformed = transform_eigen.rotation() * lin_accel
                                               - accel_unit * ang_vel_transformed.cross(ang_vel_transformed.cross(transform_eigen.translation()));

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

        // IMU input-quality monitor (HARDENING §3.3): every sample the
        // pipeline will actually consume passes through here (pre-init in any
        // mode, post-init in LIO). acc_ratio configs deliver accel in g —
        // scale to m/s^2 to match the monitor's gravity-based checks (the
        // same scaling the worker applies when it consumes the buffer).
        {
            const auto& a = imu_msg->linear_acceleration;
            const auto& w = imu_msg->angular_velocity;
            const double a_scale = acc_ratio ? 9.81 : 1.0;
            const auto& imu_rep = imu_health_.feed(
                rclcpp::Time(imu_msg->header.stamp).nanoseconds(),
                Eigen::Vector3d(w.x, w.y, w.z),
                a_scale * Eigen::Vector3d(a.x, a.y, a.z));
            imu_fault_bits_.store(imu_rep.faults, std::memory_order_relaxed);
            imu_gyro_bias_norm_.store(imu_rep.gyro_bias.norm(), std::memory_order_relaxed);
            if (!imu_rep.ok()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                    "[RESPLE] IMU health faults=0x%x (1=non-finite 2=accel-sat "
                    "4=gyro-sat 8=stuck 16=time-jump 32=noisy 64=bias-large)",
                    imu_rep.faults);
            }
        }

        // One-shot confirmation that IMU data is actually flowing — pairs
        // with the "Waiting for N IMU samples" warn so users can tell at a
        // glance whether the bag is producing IMU or the topic is wrong.
        if (!imu_first_logged_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(),
                "[RESPLE] First IMU sample received (frame_id='%s')",
                imu_msg->header.frame_id.c_str());
        }
        // Under m_buff (locked above): initialization() resolves the IMU
        // mounting rotation from this frame for gravity alignment.
        if (imu_frame_id_.empty()) {
            imu_frame_id_ = imu_msg->header.frame_id;
        }

        // HARDENING §2.3: parameterized staging cap (was hardcoded 2000),
        // drop-oldest + counted. 0 disables.
        if (max_imu_staging_ > 0 &&
            static_cast<int>(imu_int_buff.size()) >= max_imu_staging_) {
            imu_int_buff.erase(imu_int_buff.begin());
            dropped_imu_.fetch_add(1, std::memory_order_relaxed);
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
            // Transform not yet available — pass through (assumes IMU already
            // in the body frame; set q_ib/t_ib in YAML mode if it is not)
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
        // §6.3 knot under-resolution diagnostic (knot_rotation_warn_rad).
        stat.add("Knot Rotation Max (rad)",
                 knot_rot_max_mrad_.load(std::memory_order_relaxed) / 1000.0);
        stat.add("Knot Rotation Warnings",
                 static_cast<int>(knot_rot_warn_count_.load(std::memory_order_relaxed)));
        // §3.2 localizability (per-point-normalized constraint information;
        // min eigenvalue = weakest axis, condition = anisotropy; 0 until the
        // first update with >= 6 valid correspondences).
        stat.add("Localizability Min Eig (trans)", degen_min_eig_trans_.load(std::memory_order_relaxed));
        stat.add("Localizability Min Eig (rot)", degen_min_eig_rot_.load(std::memory_order_relaxed));
        stat.add("Localizability Cond (trans)", degen_cond_trans_.load(std::memory_order_relaxed));
        stat.add("Localizability Cond (rot)", degen_cond_rot_.load(std::memory_order_relaxed));
        stat.add("Loc Gate Axes (current)", loc_gate_axes_diag_.load(std::memory_order_relaxed));
        stat.add("Loc Gate Updates (cumulative)", static_cast<double>(loc_gate_updates_diag_.load(std::memory_order_relaxed)));
        stat.add("Loc Gate Cov Infl (max std m)", loc_gate_cov_infl_diag_.load(std::memory_order_relaxed));
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
        stat.add("Spline Knots (total incl. pruned)",
                 static_cast<int>(cached_spline_total_knots_.load(std::memory_order_relaxed)));
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

        // HARDENING §3.2 correspondence funnel, per diagnostics window.
        // candidates → passed_distance (full k-NN within gate) → passed_plane
        // (esti_plane accepted) → used (point-to-plane range gate, fed to the
        // IEKF). Stage-to-stage drops localize where matches are lost: sparse
        // map vs degenerate/noisy patches vs association outliers.
        {
            const uint64_t cand  = corresp_candidates_.load(std::memory_order_relaxed);
            const uint64_t knn   = corresp_passed_knn_.load(std::memory_order_relaxed);
            const uint64_t plane = corresp_passed_plane_.load(std::memory_order_relaxed);
            const uint64_t used  = corresp_used_.load(std::memory_order_relaxed);
            stat.add("Corresp Candidates (last window)",     static_cast<int>(cand  - last_corresp_candidates_));
            stat.add("Corresp Passed Distance (last window)", static_cast<int>(knn   - last_corresp_passed_knn_));
            stat.add("Corresp Passed Plane (last window)",    static_cast<int>(plane - last_corresp_passed_plane_));
            stat.add("Corresp Used in IEKF (last window)",    static_cast<int>(used  - last_corresp_used_));
            last_corresp_candidates_   = cand;
            last_corresp_passed_knn_   = knn;
            last_corresp_passed_plane_ = plane;
            last_corresp_used_         = used;
        }

        // LiDAR row gating (2026-07-26 accuracy pass). "Admitted Past nn_thresh"
        // counts rows the residual threshold rejected but the
        // `lid_cov < var_pt*coeff_cov` disjunct let in — with coeff_cov=3.0 that
        // disjunct always holds once converged, so a big number here is the
        // expected reading and quantifies what an outlier gate would have to
        // work on. The other two are 0 unless lidar_gate_sigma is set.
        stat.add("Admitted Past nn_thresh (cumulative)",
                 static_cast<int>(if_lidar_only ? estimator_lo.covEscapeAdmits()
                                                : estimator_lio.covEscapeAdmits()));
        stat.add("LiDAR Gate Rejected (cumulative)",
                 static_cast<int>(if_lidar_only ? estimator_lo.mahalRejected()
                                                : estimator_lio.mahalRejected()));
        stat.add("LiDAR Gate Disarms (cumulative)",
                 static_cast<int>(if_lidar_only ? estimator_lo.mahalDisarms()
                                                : estimator_lio.mahalDisarms()));
        // Must be 0. Non-zero means a non-finite / non-positive-diagonal
        // covariance was caught on its way out (hazard 96) — a bug, not tuning.
        const uint64_t cov_bad = cov_sanitized_.load(std::memory_order_relaxed);
        stat.add("Covariance Sanitized (cumulative, MUST be 0)",
                 static_cast<int>(cov_bad));

        // HARDENING §3.4: how many radius-prune deletions have run (0 when
        // map_prune_radius is disabled or the pose hasn't moved enough).
        stat.add("Map Radius Prunes (cumulative)",
                 static_cast<int>(radius_prune_ops_.load(std::memory_order_relaxed)));

        // HARDENING §2.3: drop-oldest counters (non-zero means the caps are
        // engaging — the worker is not keeping up with the sensors).
        stat.add("Scans Dropped (cumulative)",
                 static_cast<int>(dropped_scans_.load(std::memory_order_relaxed)));
        stat.add("IMU Samples Dropped (cumulative)",
                 static_cast<int>(dropped_imu_.load(std::memory_order_relaxed)));

        // Overload / real-time health (2026-07-07 hardening). rt_factor is
        // spline data-time advance / wall time over a ~2 s worker window; with
        // latency shedding active it stays ~1 BY DESIGN (data time is fast-
        // forwarded), so read it TOGETHER with the shed counter to tell
        // "keeping up" from "keeping up by dropping". Backlog is recomputed
        // here (not the worker's cached value) so a fully stalled worker
        // still shows it growing.
        const float rt_factor = rt_factor_.load(std::memory_order_relaxed);
        const uint64_t shed = shed_scans_.load(std::memory_order_relaxed);
        float backlog_ms = 0.f;
        {
            const int64_t newest_proc = newest_processed_ns_.load(std::memory_order_relaxed);
            if (newest_proc > 0) {
                int64_t newest_arrived = 0;
                for (const auto& [bn, bd] : lidars_data) {
                    newest_arrived = std::max(
                        newest_arrived, bd.last_t_ns.load(std::memory_order_relaxed));
                }
                if (newest_arrived > newest_proc) {
                    backlog_ms = static_cast<float>(newest_arrived - newest_proc) / 1e6f;
                }
            }
        }
        stat.add("RT Factor (data/wall, ~2s window)", static_cast<double>(rt_factor));
        stat.add("Backlog (ms, data time)", static_cast<double>(backlog_ms));
        stat.add("Scans Shed by Latency Budget (cumulative)", static_cast<int>(shed));
        stat.add("Worker Cycle Overruns (cumulative)",
                 static_cast<int>(cycle_overruns_.load(std::memory_order_relaxed)));
        // Escalation: rt_factor is only meaningful once a measurement window
        // has completed (0 until then) and frames advanced this period.
        if (frame_count > 0 && rt_factor > 0.f) {
            if (rt_factor < 0.7f) {
                level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                msg = "Real-time factor critically low - worker falling behind the stream";
            } else if (rt_factor < 0.9f
                       && level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
                level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                msg = "Real-time factor below 0.9 - worker not keeping up";
            }
        }
        // Backlog vs the latency budget (only when the budget is armed; with
        // it off there is no calibrated scale for what "too much" means).
        if (max_latency_ns_ > 0 && backlog_ms > 0.f) {
            const float budget_ms = static_cast<float>(max_latency_ns_) / 1e6f;
            if (backlog_ms > 3.f * budget_ms) {
                level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                msg = "Input backlog far exceeds max_latency_ms budget";
            } else if (backlog_ms > budget_ms
                       && level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
                level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                msg = "Input backlog exceeds max_latency_ms budget";
            }
        }

        // TF ownership guard (2026-07-07): foreign publisher on the pair we
        // broadcast. ERROR while actively conflicting; WARN when new foreign
        // transforms were seen this window but the intruder has gone quiet.
        {
            const uint64_t tf_same = tf_own_monitor_.foreignSamePair();
            const uint64_t tf_parent = tf_own_monitor_.foreignOtherParent();
            const bool tf_active = tf_own_monitor_.foreignActiveWithin(
                    this->now().nanoseconds(), tf_conflict_quiet_ns_)
                || tf_own_monitor_.foreignStaticSeen();
            const bool tf_yielding = tf_yield_active_.load(std::memory_order_relaxed);
            stat.add("TF Foreign Publishes Same Pair (cumulative)", static_cast<int>(tf_same));
            stat.add("TF Foreign Parent Claims (cumulative)", static_cast<int>(tf_parent));
            stat.add("TF Conflict Active", tf_active);
            stat.add("TF Yielding", tf_yielding);
            const uint64_t tf_delta =
                (tf_same + tf_parent) - last_tf_foreign_total_;
            last_tf_foreign_total_ = tf_same + tf_parent;
            if (publish_tf && tf_active) {
                level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                msg = tf_yielding
                    ? "TF ownership conflict active (yielding our broadcast)"
                    : "TF ownership conflict active (two publishers on our pair)";
            } else if (publish_tf && tf_delta > 0
                       && level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
                level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                msg = "Foreign TF publisher observed this window";
            }
        }

        // HARDENING §3.3 recovery-policy observability: whether odometry/TF
        // publication is currently held, and how many covariance resets the
        // 'reset' mode has performed.
        stat.add("NIS Recovery Hold Active", nis_hold_active_.load(std::memory_order_relaxed));
        stat.add("NIS Recovery Resets (cumulative)",
                 static_cast<int>(nis_resets_.load(std::memory_order_relaxed)));

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

        // HARDENING §3.3: NIS consistency verdict + IMU input health.
        const int fh_state = filter_health_state_.load(std::memory_order_relaxed);
        const char* fh_str = (fh_state == 2) ? "DIVERGED"
                            : (fh_state == 1) ? "WARN" : "OK";
        stat.add("Filter Consistency (NIS)", fh_str);
        stat.add("NIS Window Mean (x dof, ~1 healthy)",
                 nis_window_mean_.load(std::memory_order_relaxed));
        const uint32_t imu_faults = imu_fault_bits_.load(std::memory_order_relaxed);
        stat.add("IMU Fault Bits", static_cast<int>(imu_faults));
        stat.add("IMU Gyro Bias (rad/s, stationary est.)",
                 imu_gyro_bias_norm_.load(std::memory_order_relaxed));
        // Init-time TF-vs-gravity attitude cross-check (tf_gravity_check mode);
        // a large value flags a bad imu->base_link extrinsic or non-level
        // attitude frame. -1 = not computed (other modes / no gravity window).
        const double att_delta = init_attitude_delta_deg_.load(std::memory_order_relaxed);
        if (att_delta >= 0.0) {
            stat.add("Init Attitude TF-vs-Gravity Delta (deg)", att_delta);
            if (att_delta > init_attitude_consistency_deg_
                && level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
                level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                msg = "Init attitude: TF and gravity disagree (check extrinsic/frame)";
            }
        }
        if (fh_state == 2) {
            level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            msg = "Filter divergence detected (windowed NIS)";
        } else if ((fh_state == 1 || imu_faults != 0)
                   && level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            msg = (fh_state == 1) ? "Filter consistency warning (windowed NIS)"
                                  : "IMU input faults detected";
        }

        // A covariance that had to be sanitized on its way out is an internal
        // inconsistency, not a degraded-input condition — escalate to ERROR
        // unconditionally (it outranks the NIS verdict: a NaN posterior would
        // also make NIS non-finite, and this names the actual problem).
        if (cov_bad > 0) {
            level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            msg = "Published covariance had to be sanitized (non-finite or "
                  "non-positive diagonal) — internal bug";
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
        LidarConfig& lidar = lidars.at(name);
        
        // Lookup LiDAR transform
        if(!updateLidarTransform(ouster_msg_in->header.frame_id, lidar)) return;
        
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

        // Transform point cloud to body frame (TF mode only; in YAML-only /
        // fallback mode points stay in the sensor frame and q_bl/t_bl is
        // applied per point downstream)
        if (lidar.tf_latched) {
            pcl::transformPointCloud(*pc_last, *pc_last, lidar.lidar_to_baselink);
        }

        pushScanBounded(lidar_buffs, pc_last->points, time_begin);
        lidar_buffs.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] ousterLidarCallback exception: %s", e.what());
        }
    }

    // Generic PointCloud2 ingestion: resolves x/y/z + time/intensity fields by
    // name at runtime (utils/point_cloud_adapter.h), so any sensor that
    // publishes sensor_msgs/PointCloud2 works without a hand-written point
    // struct. Per-point time is normalized to a ms offset from scan start
    // (relative or absolute-epoch source fields both supported); clouds with
    // no time field still work, just without intra-scan deskew.
    void genericLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
    {
        try {
        noteFirstLidar("PointCloud2", msg_in->header.frame_id,
                       msg_in->width * msg_in->height);
        std::string name = "PointCloud2";
        if (lidars.find(name) == lidars.end()) {
            for (const auto& [lidar_name, lidar_cfg] : lidars) {
                if (lidar_cfg.type == "PointCloud2") {
                    name = lidar_name;
                    break;
                }
            }
        }
        LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(msg_in->header.frame_id, lidar)) return;

        int64_t stamp_ns = rclcpp::Time(msg_in->header.stamp).nanoseconds();
        if (stamp_ns < time_offset) return;  // skip early sim-time messages
        int64_t time_begin = stamp_ns - time_offset;

        // Field-introspecting conversion: applies blind-radius + decimation and
        // writes the deskew ms-offset into .intensity / reflectivity into
        // .curvature (the RESPLE conventions).
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        if (!resple::ingestPointCloud2(*msg_in, lidar.blind, point_filter_num,
                                       generic_adapter_cfg_, *pc_last)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] PointCloud2 on '%s' has no usable x/y/z layout — dropped",
                lidar.topic.c_str());
            return;
        }
        if (pc_last->points.empty()) return;

        // Monotonic-time gate (same contract as the per-sensor loaders): drop
        // points that precede the newest point already buffered, and track the
        // scan's max offset to advance last_t_ns.
        LidarData& lidar_buffs = lidars_data.at(name);
        int64_t last_t_ns = lidar_buffs.last_t_ns.load();
        int64_t max_ofs_ns = 0;
        size_t write_idx = 0;
        for (size_t i = 0; i < pc_last->points.size(); ++i) {
            const pcl::PointXYZINormal& pt = pc_last->points[i];
            // Non-finite xyz would poison the ikd-Tree / IEKF downstream. Include
            // the per-point time (.intensity): a non-finite time-field value from
            // the generic adapter would make ms2ns(NaN) a UB cast below and, once
            // buffered, poison the deskew std::sort comparator (NaN breaks its
            // strict-weak-ordering). The hand-written loaders drop it implicitly
            // via their intensity>=0 gate; the generic path must gate it too.
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) ||
                !std::isfinite(pt.intensity)) {
                continue;
            }
            int64_t ofs = CommonUtils::ms2ns(pt.intensity);
            if (ofs + time_begin > last_t_ns) {
                max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                pc_last->points[write_idx++] = pt;
            }
        }
        pc_last->points.resize(write_idx);
        if (pc_last->points.empty()) return;

        // Transform point cloud to body frame (TF mode only; in YAML-only /
        // fallback mode points stay in the sensor frame and q_bl/t_bl is
        // applied per point downstream)
        if (lidar.tf_latched) {
            pcl::transformPointCloud(*pc_last, *pc_last, lidar.lidar_to_baselink);
        }

        pushScanBounded(lidar_buffs, pc_last->points, time_begin);
        lidar_buffs.last_t_ns.store(time_begin + max_ofs_ns);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[RESPLE] genericLidarCallback exception: %s", e.what());
        }
    }

    void livoxLidarCallback(const livox_ros_driver::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        try {
        noteFirstLidar("Mid70Avia", livox_msg_in->header.frame_id, livox_msg_in->point_num);
        std::string name = "Mid70Avia";
        LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id, lidar)) return;

        // NOTE (applies to all three livox callbacks): the point loop below
        // intentionally starts at i = 1 — points[0] seeds pt_pre for the
        // duplicate-point filter. Mapping.cpp's livox ingest has no pt_pre and
        // loops from 0.
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        // Belt-and-braces: gate on the actual vector size, not just the
        // point_num field. A misbehaving publisher (or transport corruption)
        // can ship point_num > 0 with an empty points vector → points[0]
        // below would be OOB. Doesn't affect Ouster (we don't take this
        // callback path), but is a real fix for the Livox variants.
        if (plsize == 0 || livox_msg_in->points.empty()) return;
        // DDS robustness: clamp to the actual vector — a torn/corrupt message can
        // ship point_num > points.size(), which would OOB the points[i] loop below.
        plsize = std::min<int>(plsize, static_cast<int>(livox_msg_in->points.size()));
        pc_last->reserve(plsize);
        const int64_t stamp_ns = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        if (stamp_ns < time_offset) return;  // skip early sim-time messages (matches Ouster/generic)
        int64_t time_begin = stamp_ns - time_offset;
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

        // Transform point cloud to body frame (TF mode only; see 3a note)
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());
        if (lidar.tf_latched) {
            pcl::transformPointCloud(*pc_last, *pc_transformed, lidar.lidar_to_baselink);
        } else {
            pc_transformed = pc_last;
        }

        pushScanBounded(lidar_buffs, pc_transformed->points, time_begin);
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
        LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id, lidar)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        // Belt-and-braces: gate on the actual vector size, not just the
        // point_num field. A misbehaving publisher (or transport corruption)
        // can ship point_num > 0 with an empty points vector → points[0]
        // below would be OOB. Doesn't affect Ouster (we don't take this
        // callback path), but is a real fix for the Livox variants.
        if (plsize == 0 || livox_msg_in->points.empty()) return;
        // DDS robustness: clamp to the actual vector — a torn/corrupt message can
        // ship point_num > points.size(), which would OOB the points[i] loop below.
        plsize = std::min<int>(plsize, static_cast<int>(livox_msg_in->points.size()));
        pc_last->reserve(plsize);
        const int64_t stamp_ns = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        if (stamp_ns < time_offset) return;  // skip early sim-time messages (matches Ouster/generic)
        int64_t time_begin = stamp_ns - time_offset;
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

        // Transform point cloud to body frame (TF mode only; see 3a note)
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());
        if (lidar.tf_latched) {
            pcl::transformPointCloud(*pc_last, *pc_transformed, lidar.lidar_to_baselink);
        } else {
            pc_transformed = pc_last;
        }

        pushScanBounded(lidar_buffs, pc_transformed->points, time_begin);
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
        LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id, lidar)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        int plsize = livox_msg_in->point_num;
        // Belt-and-braces: gate on the actual vector size, not just the
        // point_num field. A misbehaving publisher (or transport corruption)
        // can ship point_num > 0 with an empty points vector → points[0]
        // below would be OOB. Doesn't affect Ouster (we don't take this
        // callback path), but is a real fix for the Livox variants.
        if (plsize == 0 || livox_msg_in->points.empty()) return;
        // DDS robustness: clamp to the actual vector — a torn/corrupt message can
        // ship point_num > points.size(), which would OOB the points[i] loop below.
        plsize = std::min<int>(plsize, static_cast<int>(livox_msg_in->points.size()));
        pc_last->reserve(plsize);
        const int64_t stamp_ns = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        if (stamp_ns < time_offset) return;  // skip early sim-time messages (matches Ouster/generic)
        int64_t time_begin = stamp_ns - time_offset;
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

        // Transform point cloud to body frame (TF mode only; see 3a note)
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());
        if (lidar.tf_latched) {
            pcl::transformPointCloud(*pc_last, *pc_transformed, lidar.lidar_to_baselink);
        } else {
            pc_transformed = pc_last;
        }

        pushScanBounded(lidar_buffs, pc_transformed->points, time_begin);
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
        LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(hesai_msg_in->header.frame_id, lidar)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::PointCloud<hesai_ros::Point>::Ptr pc_last_hesai(new pcl::PointCloud<hesai_ros::Point>());
        pcl::fromROSMsg(*hesai_msg_in, *pc_last_hesai);
        size_t plsize = pc_last_hesai->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(hesai_msg_in->header.stamp);
        const int64_t stamp_ns = timestamp_begin.nanoseconds();
        if (stamp_ns < time_offset) return;  // skip early sim-time messages (matches Ouster/generic)
        int64_t time_begin = stamp_ns - time_offset;
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

        // Transform point cloud to body frame (TF mode only; see 3a note)
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());
        if (lidar.tf_latched) {
            pcl::transformPointCloud(*pc_last, *pc_transformed, lidar.lidar_to_baselink);
        } else {
            pc_transformed = pc_last;
        }

        pushScanBounded(lidar_buffs_hesai, pc_transformed->points, time_begin);
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
        LidarConfig& lidar = lidars.at(name);

        if(!updateLidarTransform(livox_msg_in->header.frame_id, lidar)) return;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::PointCloud<livox_mid360_boxi::Point>::Ptr pc_last_livox(new pcl::PointCloud<livox_mid360_boxi::Point>());
        pcl::fromROSMsg(*livox_msg_in, *pc_last_livox);
        size_t plsize = pc_last_livox->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(livox_msg_in->header.stamp);
        const int64_t stamp_ns = timestamp_begin.nanoseconds();
        if (stamp_ns < time_offset) return;  // skip early sim-time messages (matches Ouster/generic)
        int64_t time_begin = stamp_ns - time_offset;
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

        // Transform point cloud to body frame (TF mode only; see 3a note)
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZINormal>());
        if (lidar.tf_latched) {
            pcl::transformPointCloud(*pc_last, *pc_transformed, lidar.lidar_to_baselink);
        } else {
            pc_transformed = pc_last;
        }

        pushScanBounded(lidar_buffs_boxi, pc_transformed->points, time_begin);
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
    // odom/dense_pub_hz interval (0 = off) and the back-fill cap after a
    // stall / NIS-hold release / bag restart. See utils/dense_pub.h.
    int64_t dense_pub_interval_ns_ = 0;
    static constexpr int64_t kDensePubMaxBackfillNs = 1000000000;  // 1 s

    void publishPoseAndTf()
    {
        const int64_t pose_time_ns = spline->maxTimeNs();
        // Skip duplicates: the inner collectMeasurements loop can fire many
        // times before the spline advances, and tf2 / nav listeners reject
        // identical-stamp transforms anyway.
        if (pose_time_ns <= last_pose_pub_time_ns_) return;
        // Dense back-fill (odom/dense_pub_hz, dliio-lessons review
        // 2026-07-10). Steady state already publishes once per knot (this
        // function runs per collectMeasurements iteration; measured ~100 Hz
        // on R_Campus baseline), so this is a FLOOR, not the normal rate:
        // whenever the edge advances by more than one knot per call (propRCP
        // jumping a scan gap, overload shedding — hazard 69 — or an NIS-hold
        // release), the duplicate gate would emit one pose at the new edge
        // and leave a hole in the published chain exactly where downstream
        // tf2 lookups are most fragile. Back-fill the segment (last publish,
        // edge) by spline interpolation — data-time stamps, never
        // extrapolated, latency unchanged; also upsamples past knot_hz if
        // configured higher. All samples in a batch reuse the current IEKF
        // posterior covariance — there is one posterior per batch and these
        // stamps belong to it. The NIS 'hold' gate is upstream at the call
        // site, so a held window is never back-filled densely either (the
        // cap bounds the burst at release).
        // One posterior per batch: getLastPoseCovariance projects the current
        // IEKF posterior through the spline Jacobian AT THE EDGE, so every
        // sample of this batch carries the same matrices by design — compute
        // them once here instead of per publishPoseAtNs call (a 6x24 Jacobian
        // product each; up to ~1000 calls in a worst-case back-fill burst).
        //
        // Pull the IEKF posterior covariance from the active estimator and
        // project it through the spline Jacobian so downstream consumers
        // (robot_localization, sierra) actually see per-step uncertainty
        // rather than the static cov_pose YAML diagonal.
        Eigen::Matrix<double, 6, 6> P_pose =
            if_lidar_only ? estimator_lo.getLastPoseCovariance()
                          : estimator_lio.getLastPoseCovariance();
        // Advisory degeneracy inflation: grow the REPORTED translation
        // uncertainty along axes the gate has suppressed on consecutive frames,
        // so a downstream EKF sees honest absolute uncertainty in a tunnel (the
        // raw IEKF posterior only reports cm-scale local uncertainty there).
        // Publish-side only — the internal filter covariance and the trajectory
        // are untouched; no-op when loc_gate_cov_rate=0. See
        // Estimator::accumGateInflation and HARDENING §6.7.
        P_pose.topLeftCorner<3, 3>() += if_lidar_only
            ? estimator_lo.locGateCovInfl()
            : estimator_lio.locGateCovInfl();
        Eigen::Matrix<double, 6, 6> P_twist =
            if_lidar_only ? estimator_lo.getLastTwistCovariance()
                          : estimator_lio.getLastTwistCovariance();

        // Last line of defence on the wire. Everything upstream is checked (LLT
        // failure -> update skipped, NIS -> divergence detector, non-finite
        // points dropped at ingest), so a bad matrix here means a bug — but
        // robot_localization inverts what we hand it, so one NaN would turn the
        // consumer's whole state NaN permanently, with no diagnostic pointing
        // back here. Assert it instead. See utils/filter_health.h.
        const auto pose_sanity = resple::health::sanitizeCovariance(P_pose);
        const auto twist_sanity = resple::health::sanitizeCovariance(P_twist);
        if (pose_sanity.modified() || twist_sanity.modified()) {
            cov_sanitized_.fetch_add(1, std::memory_order_relaxed);
            if (pose_sanity.replaced || twist_sanity.replaced) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[RESPLE] non-finite %s%s covariance replaced with an "
                    "uninformative diagonal before publish — this is a bug; "
                    "check the IEKF posterior",
                    pose_sanity.replaced ? "pose" : "",
                    twist_sanity.replaced ? (pose_sanity.replaced ? "+twist" : "twist") : "");
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RESPLE] floored %d pose / %d twist covariance diagonal "
                    "entries to a positive minimum before publish",
                    pose_sanity.floored, twist_sanity.floored);
            }
        }

        for (int64_t t_ns = resple::densepub::firstDenseSampleNs(
                 last_pose_pub_time_ns_, pose_time_ns, dense_pub_interval_ns_,
                 kDensePubMaxBackfillNs);
             t_ns < pose_time_ns; t_ns += dense_pub_interval_ns_) {
            publishPoseAtNs(t_ns, P_pose, P_twist);
        }
        publishPoseAtNs(pose_time_ns, P_pose, P_twist);
        last_pose_pub_time_ns_ = pose_time_ns;
    }

    void publishPoseAtNs(int64_t pose_time_ns,
                         const Eigen::Matrix<double, 6, 6>& P_pose,
                         const Eigen::Matrix<double, 6, 6>& P_twist)
    {
        Eigen::Vector3d t_pose = spline->itpPosition(pose_time_ns);
        Eigen::Quaterniond q_pose;
        spline->itpQuaternion(pose_time_ns, &q_pose);

        geometry_msgs::msg::PoseStamped pose_msg =
            CommonUtils::pose2msg(odom_id, pose_time_ns, t_pose, q_pose);
        pub_pose->publish(pose_msg);

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
            // TF ownership guard 'yield': while a foreign publisher on our
            // pair was seen within the quiet window, suspend OUR broadcast
            // (odometry topics above keep flowing) instead of fighting it —
            // interleaved publishers make the pose flicker. Resumes
            // automatically once the intruder goes quiet.
            // Sticky static claims hold the yield regardless of the quiet
            // window: a foreign /tf_static transform lives in consumers'
            // buffers forever (cleared only by a lifecycle re-configure).
            const bool yield_now = tf_conflict_action_ == "yield"
                && (tf_own_monitor_.foreignActiveWithin(this->now().nanoseconds(), tf_conflict_quiet_ns_)
                    || tf_own_monitor_.foreignStaticSeen());
            tf_yield_active_.store(yield_now, std::memory_order_relaxed);
            if (yield_now) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RESPLE] yielding the %s->%s TF to a foreign publisher "
                    "(tf_conflict_action=yield); odometry topics continue.",
                    tf_own_monitor_.parent().c_str(), tf_own_monitor_.child().c_str());
                return;
            }
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
            // Record the exact wire stamp BEFORE sending so the DDS loopback
            // of our own message can never race the ring insert.
            tf_own_monitor_.notePublished(
                rclcpp::Time(transformStamped.header.stamp).nanoseconds());
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
            // The drop above can empty a lagging lidar's pt_buff entirely (its
            // whole backlog was older than the global cutoff — e.g. one LiDAR
            // >200 ms behind the freshest in a multi-LiDAR config, or a
            // sub-5 Hz sensor whose next scan is still in pc_buff). front()
            // below would be UB on an empty deque; wait for the next cycle to
            // refill instead (bug-hunt 2026-07-02 finding #11).
            for (const auto& [lidar_name, lidar_data] : lidars_data) {
                if (lidar_data.pt_buff.empty()) {
                    return false;
                }
            }
        }
        int64_t start_t_ns = std::numeric_limits<int64_t>::max();
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            start_t_ns = std::min(start_t_ns, std::max(lidar_data.pt_buff.front().time_ns, int64_t(0)));
        }
        if (!if_init_filter) {
            Eigen::Quaterniond q_WI = Eigen::Quaterniond::Identity();

            // LO mode can run without any IMU (upstream never required one):
            // if too few samples arrive within lo_imu_wait_timeout_, proceed
            // with identity orientation instead of blocking initialization
            // forever (bug-hunt 2026-07-02 finding #10). LIO still requires
            // IMU unconditionally.
            bool use_imu_gravity = true;
            if (if_lidar_only) {
                std::lock_guard<std::mutex> lk(m_buff);
                if (static_cast<int>(imu_buff.size()) < imu_init_num_samples_) {
                    // NODE-CLOCK wait (2026-07-08 clock-domain audit): sim-time
                    // aware so throttled replay can't expire it before the bag
                    // delivers the IMU samples.
                    if (lo_imu_wait_.elapsedSec(this->now().nanoseconds())
                            > lo_imu_wait_timeout_) {
                        use_imu_gravity = false;
                        RCLCPP_WARN(this->get_logger(),
                            "[RESPLE] LO mode: insufficient IMU after %.1f s — initializing "
                            "without gravity alignment (identity orientation; map z axis "
                            "is the sensor z axis)", lo_imu_wait_timeout_);
                    }
                }
            }

            // ============ Initial attitude (Design B) ========================
            // Two candidate sources for the world<-base_link initial rotation;
            // policy in init_attitude_source (see member decls). The world is
            // gravity-aligned by construction (Z up) on every path.
            const bool tf_mode = (init_attitude_source_ == "tf" ||
                                  init_attitude_source_ == "tf_gravity_check");
            const bool check_mode = (init_attitude_source_ == "tf_gravity_check");

            // ---- Gravity candidate (accelerometer) --------------------------
            // In gravity mode this BLOCKS (return false → retry) until a
            // stationary window exists — the safe self-contained default. In
            // tf modes it is OPPORTUNISTIC: insufficient samples or a noisy
            // window just leave grav_ok=false and init proceeds via TF.
            bool grav_ok = false;
            Eigen::Quaterniond q_WI_grav = Eigen::Quaterniond::Identity();
            double accel_variance = 0.0;
            int n_imu = 0;
            if (use_imu_gravity) {
                Eigen::Vector3d gravity_mean = Eigen::Vector3d::Zero();
                bool window_ok = false;
                {
                    std::unique_lock<std::mutex> imu_lock(m_buff);
                    // Bound the pre-init staging (finding #13): the
                    // quietest-window scan below is O(windows x n_imu), and
                    // imu_buff otherwise grows unbounded while the variance
                    // gate keeps failing (robot in motion). Keep a recent
                    // multiple of the window — a stationary interval must be
                    // recent to be a valid gravity anchor.
                    const int init_buff_cap = std::max(4 * imu_init_num_samples_, 400);
                    while (static_cast<int>(imu_buff.size()) > init_buff_cap) {
                        imu_buff.pop_front();
                    }
                    const int buff_size = imu_buff.size();

                    if (buff_size < imu_init_num_samples_) {
                        if (!tf_mode) {
                            imu_lock.unlock();
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Waiting for %d IMU samples for gravity alignment (current: %d)",
                                imu_init_num_samples_, buff_size);
                            return false;   // gravity mode: keep waiting
                        }
                        // tf mode: gravity unavailable this cycle, fall through.
                    } else {
                        if (!imu_count_logged_.exchange(true)) {
                            RCLCPP_INFO(this->get_logger(),
                                "[RESPLE] Collected %d IMU samples for gravity alignment "
                                "(buffer size: %d)", imu_init_num_samples_, buff_size);
                        }
                        // Scan all n_imu-wide windows, pick the quietest — if
                        // the bag started mid-motion the tail would fail the
                        // variance check and anchor the map to a noisy gravity.
                        n_imu = std::min(imu_init_num_samples_, buff_size);
                        const int n_windows = buff_size - n_imu + 1;
                        Eigen::Vector3d best_mean = Eigen::Vector3d::Zero();
                        double best_var = std::numeric_limits<double>::infinity();
                        int best_start = 0;
                        for (int w = 0; w < n_windows; ++w) {
                            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                            for (int i = 0; i < n_imu; ++i) sum += imu_buff.at(w + i).accel;
                            const Eigen::Vector3d mean = sum / n_imu;
                            double var = 0.0;
                            for (int i = 0; i < n_imu; ++i)
                                var += (imu_buff.at(w + i).accel - mean).squaredNorm();
                            var /= n_imu;
                            if (var < best_var) { best_var = var; best_mean = mean; best_start = w; }
                        }
                        gravity_mean = best_mean;
                        accel_variance = best_var;

                        if (accel_variance > imu_init_max_variance_) {
                            if (!tf_mode) {
                                imu_lock.unlock();
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "IMU readings too noisy for initialization (best window variance: "
                                    "%.4f > %.4f, scanned %d windows). Ensure robot is stationary at "
                                    "some point during launch or increase imu_init_max_variance.",
                                    accel_variance, imu_init_max_variance_, n_windows);
                                return false;   // gravity mode: wait for stationary
                            }
                            // tf mode: noisy window → gravity unavailable.
                        } else {
                            RCLCPP_INFO(this->get_logger(),
                                "[RESPLE] gravity window: picked samples [%d..%d] of %d "
                                "(variance %.4f, scanned %d windows)",
                                best_start, best_start + n_imu - 1, buff_size,
                                accel_variance, n_windows);
                            window_ok = true;
                        }
                    }
                }
                if (window_ok) {
                    // Rotate sensor-frame gravity into base_link when the IMU
                    // mounting rotation is known (finding #9): pre-init samples
                    // are buffered RAW. In YAML mode this rotates only when
                    // q_ib/t_ib is configured (identity default = no-op).
                    if (updateImuTransform(imu_frame_id_)) {
                        const Eigen::Quaterniond R_bi(
                            tf2::transformToEigen(imu_to_baselink_).rotation());
                        gravity_mean = R_bi * gravity_mean;
                    }
                    // g2R already removes yaw (gravity cannot observe it).
                    q_WI_grav = Quater::positify(Eigen::Quaterniond(CommonUtils::g2R(gravity_mean)));
                    grav_ok = true;
                }
            }

            // ---- TF candidate (base_link attitude in a gravity-aligned frame)
            bool tf_ok = false;
            Eigen::Quaterniond q_WI_tf = Eigen::Quaterniond::Identity();
            Eigen::Vector3d tf_ypr = Eigen::Vector3d::Zero();
            if (tf_mode) {
                Eigen::Quaterniond q_wb;
                if (lookupInitAttitude(q_wb)) {
                    tf_ypr = CommonUtils::R2ypr(q_wb.toRotationMatrix());
                    Eigen::Vector3d ypr_use = tf_ypr;
                    if (!init_yaw_from_tf_) ypr_use.x() = 0.0;  // yaw stays a free gauge
                    q_WI_tf = Quater::positify(Eigen::Quaterniond(CommonUtils::ypr2R(ypr_use)));
                    tf_ok = true;
                }
            }

            // ---- Select + cross-check ---------------------------------------
            if (tf_ok) {
                q_WI = q_WI_tf;
                if (check_mode && grav_ok) {
                    const Eigen::Vector3d g_ypr = CommonUtils::R2ypr(q_WI_grav.toRotationMatrix());
                    const double dpitch = tf_ypr.y() - g_ypr.y();
                    const double droll  = tf_ypr.z() - g_ypr.z();
                    const double delta_deg = std::sqrt(dpitch * dpitch + droll * droll);
                    init_attitude_delta_deg_.store(delta_deg, std::memory_order_relaxed);
                    if (delta_deg > init_attitude_consistency_deg_) {
                        RCLCPP_WARN(this->get_logger(),
                            "[RESPLE] init attitude: TF vs gravity roll/pitch disagree by %.2f deg "
                            "(> %.2f). Check the imu->base_link extrinsic and that '%s' is "
                            "gravity-aligned. Using TF.",
                            delta_deg, init_attitude_consistency_deg_, init_attitude_frame_.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                            "[RESPLE] init attitude: TF vs gravity consistent (%.2f deg).", delta_deg);
                    }
                }
                RCLCPP_INFO(this->get_logger(),
                    "[RESPLE] initial attitude from TF '%s' (yaw %s, mode: %s)",
                    init_attitude_frame_.c_str(), init_yaw_from_tf_ ? "from TF" : "free",
                    if_lidar_only ? "LO" : "LIO");
            } else if (grav_ok) {
                q_WI = q_WI_grav;
                if (tf_mode) {
                    RCLCPP_WARN(this->get_logger(),
                        "[RESPLE] init attitude: TF '%s' unavailable; using gravity alignment.",
                        init_attitude_frame_.c_str());
                }
                RCLCPP_INFO(this->get_logger(),
                    "Gravity alignment successful (samples: %d, variance: %.4f, mode: %s)",
                    n_imu, accel_variance, if_lidar_only ? "LO" : "LIO");
            } else {
                // Neither source available this cycle. In tf mode, give the TF
                // a bounded chance (tf_wait_timeout) before falling to identity,
                // so a slow static-TF publisher doesn't init the map tilted.
                if (tf_mode) {
                    // NODE-CLOCK wait (2026-07-08 clock-domain audit; see
                    // lo_imu_wait_ above for the rationale).
                    if (init_attitude_wait_.elapsedSec(this->now().nanoseconds())
                            <= tf_wait_timeout_) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "[RESPLE] waiting for init attitude: TF '%s' -> '%s' (no clean gravity "
                            "window yet)", init_attitude_frame_.c_str(), frame_id.c_str());
                        return false;
                    }
                    RCLCPP_WARN(this->get_logger(),
                        "[RESPLE] init attitude: no TF and no stationary gravity window after %.1f s; "
                        "identity orientation (map z = base_link z).", tf_wait_timeout_);
                }
                q_WI = Eigen::Quaterniond::Identity();
            }

            // World is gravity-aligned by construction (Z up). With
            // gravity_magnitude=9.81 this equals the old q_WI·(ĝ·9.81), which
            // is exactly [0,0,9.81] since g2R maps ĝ→+Z.
            gravity = Eigen::Vector3d(0.0, 0.0, gravity_magnitude_);

            // Initialize the filter BEFORE flipping if_init_filter — this way
            // when the callback acquires m_buff and sees if_init_filter=true,
            // the SplineState is already fully constructed.
            initFilter(start_t_ns, Eigen::Vector3d(0, 0, 0), q_WI);

            // Flip if_init_filter under m_buff so getImuCallback's snapshot
            // observes a coherent pre→post transition. The IMU buffers are
            // cleared in BOTH modes here — callbacks that already acquired
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
                // Purge ALL pre-init IMU staged for gravity alignment (the
                // full buffers — no pre-start trim is needed first, the clear
                // subsumes it). Those samples were pushed RAW (sensor frame)
                // — getImuCallback applies the base_link extrinsic
                // (transformImu) only POST-init — and feeding them to the
                // IEKF (which treats imu_meas as base_link-frame) would fuse
                // the first ~scan of IMU in the sensor frame: a systematic
                // accel/gyro-direction error at startup scaling with the IMU
                // mounting rotation (finding #9 corrected only the
                // gravity-init MEAN, not the per-sample data actually fed to
                // the filter). LO disables IMU hereafter; LIO resumes with
                // transformImu-corrected samples on the next callbacks, so
                // the spline's earliest window [start_t_ns, first post-init
                // sample] is fused LiDAR-only (up to ~a scan of staged
                // backlog) — a deliberate trade of a short IMU gap for a
                // correct frame. Do NOT make this clear conditional again:
                // that reintroduces hazard 79.
                imu_int_buff.clear();
                imu_buff.clear();
                if (if_lidar_only) {
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
        // No configured LiDARs → nothing to segment. Reachable only by a
        // zombie map-update lambda that acquires mtx_map_ AFTER on_cleanup
        // cleared `lidars`: without this guard the loop below would leave the
        // ±DBL_MAX position sentinels untouched and, if it also observed the
        // post-cleanup localmap_initialized_==false, latch garbage inverted
        // cube vertices that silently disable FOV segmentation for the NEXT
        // activation (2026-07-14 review). Live nodes always have ≥1 lidar.
        if (lidars.empty()) {
            return;
        }
        // clear(), not just shrink_to_fit(). cub_needrm is a node MEMBER, and
        // shrink_to_fit only trims capacity — it keeps every element. So each
        // cube move re-submitted the entire history of FOV-trim boxes to
        // Delete_Point_Boxes, not just this move's trailing slabs. Upstream
        // FAST-LIO's lasermap_fov_segment() opens with cub_needrm.clear();
        // that line was lost here.
        //
        // Consequence on a REVISITED traverse: a box recorded while trimming
        // behind the platform on the outbound leg lies INSIDE the live cube
        // once the platform turns around, so the stale box deletes map the
        // filter is currently localizing against. Verified against the real
        // vendored ikd-Tree on an out-and-back run; needs roughly three cube
        // shifts out (~950 m at the shipped cube_len 1000 / det_range 100)
        // plus a two-shift return before a stale box reaches the live cube,
        // and k-NN recovers on the next mapIncremental — so it is a
        // one-frame degradation per cube move, not a permanent hole. The
        // unbounded O(moves^2) growth of the delete list, all under mtx_map_
        // UNIQUE (which blocks the IEKF's k-NN), is the other half.
        cub_needrm.clear();
        cub_needrm.shrink_to_fit();
        Eigen::Vector3d pos_lidar_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        // lowest(), not min(): numeric_limits<double>::min() is the smallest
        // POSITIVE normal (~2.2e-308), not the most negative double. With
        // min() the .max() reduction below silently fails for any pos with
        // negative components, biasing the local-map cube ~500m off-axis.
        Eigen::Vector3d pos_lidar_max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest());
        {
            // Read the spline (maxTimeNs + getPositionLiDAR's itpPosition/
            // itpQuaternion) under spline_mutex_. lasermapFovSegment runs in the
            // async map-update task holding mtx_map_ UNIQUE, but the worker grows
            // the spline (collectMeasurements -> addOneStateKnot) under
            // spline_mutex_ ALONE (no mtx_map_) — so mtx_map_ does not exclude
            // that write and the previously-unlocked read here raced with it
            // (TSan-flagged). Lock order mtx_map_ -> spline_mutex_ is preserved
            // (the caller already holds mtx_map_ unique). getPositionLiDAR is
            // pure-read and takes no lock itself, so no double-lock.
            std::lock_guard<std::mutex> spline_lock(spline_mutex_);
            for (const auto& [lidar_name, lidar] : lidars) {
                Eigen::Vector3d pos_lidar = getPositionLiDAR(spline->maxTimeNs(), lidar.t_bl);
                pos_lidar_min = pos_lidar_min.array().min(pos_lidar.array()).matrix();
                pos_lidar_max = pos_lidar_max.array().max(pos_lidar.array()).matrix();
            }
        }
        if (!localmap_initialized_){
            for (int i = 0; i < 3; i++){
                LocalMap_Points.vertex_min[i] = pos_lidar_min(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_lidar_max(i) + cube_len / 2.0;
            }
            localmap_initialized_ = true;
            return;
        }
        // HARDENING §3.4: radius-based pruning runs every tick (it has its
        // own movement hysteresis), independent of the cube's edge trigger
        // below — the cube only deletes when the pose nears its edge, so
        // wandering inside a large cube otherwise keeps stale far geometry
        // alive for the whole run.
        pruneMapRadius(0.5 * (pos_lidar_min + pos_lidar_max));
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

    // HARDENING §3.4: radius-based map pruning (supplement to the FOV cube).
    // When map_prune_radius > 0, keep only the axis-aligned box of half-extent
    // R centered on the current lidar pose: on first activation, and whenever
    // the pose has moved more than 10% of R since the last prune, delete the
    // slabs of the local-map cube that lie outside that box. R is floored at
    // 2x det_range so pruning can never bite into the sensor's live
    // measurement sphere (matches in findCorresp are all within det_range).
    //
    // Threading: called only from lasermapFovSegment, i.e. inside the async
    // map-update task under mtx_map_ unique; successive tasks are serialized
    // by the worker's future wait, so the plain (non-atomic) prune-state
    // members are single-threaded.
    void pruneMapRadius(const Eigen::Vector3d& center)
    {
        if (map_prune_radius_ <= 0.0) {
            return;
        }
        const double R = std::max(map_prune_radius_, 2.0 * double(det_range));
        if (radius_prune_initialized_ &&
            (center - last_radius_prune_center_).norm() < 0.1 * R) {
            return;
        }
        // Every live map point lies inside LocalMap_Points (mapIncremental
        // only adds within the FOV cube), so carving cube \ keep into at most
        // 6 disjoint slabs covers everything outside the retention box. The
        // decomposition lives in the unit-tested geometry core.
        resple::geom::AabBox outer, keep;
        for (int i = 0; i < 3; i++) {
            outer.min(i) = LocalMap_Points.vertex_min[i];
            outer.max(i) = LocalMap_Points.vertex_max[i];
            keep.min(i) = center(i) - R;
            keep.max(i) = center(i) + R;
        }
        const std::vector<resple::geom::AabBox> slabs = resple::geom::subtractBox(outer, keep);
        if (!slabs.empty()) {
            std::vector<BoxPointType> rm;
            rm.reserve(slabs.size());
            for (const auto& s : slabs) {
                BoxPointType b;
                for (int i = 0; i < 3; i++) {
                    b.vertex_min[i] = static_cast<float>(s.min(i));
                    b.vertex_max[i] = static_cast<float>(s.max(i));
                }
                rm.push_back(b);
            }
            ikdtree.Delete_Point_Boxes(rm);
            radius_prune_ops_.fetch_add(1, std::memory_order_relaxed);
        }
        last_radius_prune_center_ = center;
        radius_prune_initialized_ = true;
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

    // Overload hardening (2026-07-07): 0 (default) keeps rclcpp's behaviour
    // (one executor thread per core), which oversubscribes small shared
    // machines — the sensor group is MutuallyExclusive so >2 threads buy
    // almost nothing. Low-resource profile sets 2.
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
