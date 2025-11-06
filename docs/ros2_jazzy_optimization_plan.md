# ROS 2 Jazzy Optimization Plan for RESPLE

This plan is tailored to the current repository layout and code. It is organized by phases with rationale, expected impact, and concrete tasks you can check off.

## Environment
- Platform: WSL2 (Ubuntu 24.04) on Windows 11
- ROS 2 Distribution: Jazzy Jalisco
- Workspace: ~/ros2_ws
- Source location: ~/ros2_ws/src/RESPLE

## Quick start commands
```bash
# Source ROS 2 and workspace (add to ~/.bashrc for persistence)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Build workspace
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to resple

# Run nodes (headless, no RViz)
ros2 run resple RESPLE --ros-args --params-file $(ros2 pkg prefix resple)/share/resple/config/config_ouster.yaml
ros2 run resple Mapping --ros-args --params-file $(ros2 pkg prefix resple)/share/resple/config/config_ouster.yaml

# Or use launch file (with RViz if display is configured)
ros2 launch resple resple_ouster.launch.py
```

## Repo snapshot highlights
- Packages: resple (C++ node + mapping) and 4 msg packages (estimate_msgs, AviaResple_msgs, HAP360_msgs, Mid70Avia_msgs).
- Core deps: rclcpp, tf2_ros, PCL, Eigen, OpenMP; custom Livox/Hesai interfaces.
- resple/src: heavy point-cloud processing, custom ikd-Tree, OpenMP loops, manual worker thread; publishers/subscriptions created with integer queue size (no explicit QoS); not using components/intra-process; CMake uses global flags (-O3 -march=native -fopenmp).

## Priorities at a glance
- P0 (must): Correctness, cross-platform build hygiene, QoS and executor fixes, containerization update to Jazzy.
- P1 (should): Composition + intra-process, loaned messages, data-path copy reductions, tracing & perf harness.
- P2 (could): Algorithmic micro-optimizations, dynamic runtime tuning, optional SIMD/arch flags.

## Phase 0 — Baseline + profiling (P0)

**Current status**: WSL2 environment ready; workspace built with Release config.

### Tasks
- [ ] Add a RelWithDebInfo default build and verify runtime parity vs Release.
  ```bash
  cd ~/ros2_ws
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select resple
  ```
- [ ] Create a perf launch with representative bag(s) and record baseline: CPU %, latency to publish pose/scan, memory RSS.
  ```bash
  # In one terminal
  ros2 launch resple resple_ouster.launch.py
  # In another terminal
  ros2 topic hz /pose & ros2 topic hz /current_scan &
  top -p $(pgrep -d',' RESPLE)
  ```
- [ ] Enable topic statistics on heavy topics (PointCloud2, Estimate) with rclcpp TopicStatistics.
- [ ] Add tracetools baseline trace around main loops: processData, mapIncremental, Mapping::process.
  ```bash
  ros2 trace --session-name resple_baseline ~/ros2_ws/traces
  # Run workload, then analyze with:
  tracecompass ~/ros2_ws/traces
  ```
- [ ] Git commit checkpoint: `git add -A && git commit -m "RESPLE Jazzy: Phase 0 - baseline profiling and tracing"`

## Phase 1 — Build system modernization (P0)

### Tasks
- [ ] Replace global CMAKE_CXX_FLAGS with target-based properties:
  - set(CMAKE_CXX_STANDARD 17)
  - target_compile_features(... PRIVATE cxx_std_17)
  - target_compile_options with generator expressions per compiler (GCC/Clang vs MSVC).
  - Link OpenMP via target_link_libraries(... PRIVATE OpenMP::OpenMP_CXX); drop FindOpenMP/CFLAGS tweaks.
- [ ] Provide cache options:
  - option(ENABLE_OPENMP "Enable OpenMP parallelism" ON)
  - option(ENABLE_NATIVE_ARCH "Use -march=native or /arch:AVX2" OFF)
- [ ] Move -DEIGEN_INITIALIZE_MATRICES_BY_NAN to Debug only; ensure NDEBUG in Release/RelWithDebInfo.
- [ ] Avoid double-compiling same sources:
  - Make library target contain common sources; executables link to the library only (do not also compile src/*.cpp into the exe).
- [ ] Remove message_runtime from resple/package.xml export; depend on rosidl_default_runtime only in msg packages.
- [ ] Bump cmake_minimum_required to ≥3.16 (consistent with modern ROS 2 templates).
- [ ] Git commit checkpoint: `git add -A && git commit -m "RESPLE Jazzy: Phase 1 - CMake modernization and package hygiene"`

## Phase 2 — Executor, QoS, composition (P0→P1)

### Tasks
- [ ] Replace manual spin_some + worker thread with a MultiThreadedExecutor and callback groups.
- [ ] Make both RESPLE and Mapping composable nodes (rclcpp_components). Provide a composed launch to run them in one process.
- [ ] Enable intra-process comms (NodeOptions().use_intra_process_comms(true)) in composed mode to cut copies.
- [ ] Replace integer queue args with explicit SensorDataQoS and KeepLast profiles:
  - Subscriptions to PointCloud2/IMU: rclcpp::SensorDataQoS().best_effort().keep_last(10)
  - Estimation/pose topics: reliability and history as appropriate (often reliable + keep_last small).
- [ ] Git commit checkpoint: `git add -A && git commit -m "RESPLE Jazzy: Phase 2 - executor, QoS, composition, intra-process"`

## Phase 3 — Zero-copy and data-path reductions (P1)

### Tasks
- [ ] Use loaned messages for publishing large PointCloud2 where supported:
  - borrow_loaned_message(), fill fields, publish(std::move(loaned_msg)).
- [ ] Minimize PCL↔ROS conversions:
  - Where possible, construct sensor_msgs::msg::PointCloud2 directly using PointCloud2Iterator.
- [ ] Pre-allocate and reuse buffers:
  - Reuse pcl::PointCloud buffers (reserve + clear) where allocations remain hot; already partially done, extend across code paths.
  - Replace aligned_deque for pt_meas and nearest-points with ring buffers to reduce allocations.
- [ ] Parameterize NUM_OF_THREAD and NUM_MATCH_POINTS; set via parameters; default to std::thread::hardware_concurrency for OpenMP.
- [ ] Git commit checkpoint: `git add -A && git commit -m "RESPLE Jazzy: Phase 3 - zero-copy and data-path reductions"`

## Phase 4 — Algorithmic and concurrency tuning (P1)

### Tasks
- [ ] Parallelize additional hotspots where safe (e.g., MappingBase::transformCloud loop) with OpenMP and proper scheduling.
- [ ] Make voxel sizes adaptive based on load (increase leaf size when callback lag grows).
- [ ] Tune ikd-Tree update batches: coalesce PointToAdd/PointNoNeedDownsample writes to improve cache locality.
- [ ] Use unordered_map for lidars/lidars_data (small win, optional) or keep as map since cardinality is small.
- [ ] Git commit checkpoint: `git add -A && git commit -m "RESPLE Jazzy: Phase 4 - algorithmic and concurrency tuning"`

## Phase 5 — Observability, tests, CI (P1)

### Tasks
- [ ] Add ros2_tracing spans for callbacks and critical sections; export trace presets.
- [ ] Add an ament_add_gtest micro-benchmark (or Google Benchmark) for key loops (association, map update, transformCloud).
- [ ] Add ament linters: clang-tidy (performance-* set), cppcheck; fix low-hanging perf warnings.
- [ ] Set up CI to build and test on Jazzy; run basic perf smoke with recorded subset bag.
- [ ] Git commit checkpoint: `git add -A && git commit -m "RESPLE Jazzy: Phase 5 - observability, tests, CI"`

## Phase 6 — Optional arch-specific speedups (P2)

### Tasks
- [ ] Gated AVX2/AVX512 builds (ENABLE_NATIVE_ARCH or /arch:AVX2) with runtime checks; ensure portable default remains ON by default.
- [ ] Consider PCL compiler definitions to favor vectorization; ensure Eigen uses -DEIGEN_MAX_ALIGN_BYTES=64 where useful.
- [ ] Git commit checkpoint: `git add -A && git commit -m "RESPLE Jazzy: Phase 6 - arch-specific speedups"`

## Concrete file-level recommendations
- resple/CMakeLists.txt
  - Remove hard-coded flags line; migrate to target_compile_options and target_link_libraries(OpenMP::OpenMP_CXX) conditionally.
  - Don't compile src/*.cpp directly into executables if the same files are in the resple library; link only.
  - ament_export_dependencies(...) should not list ament_lint_auto; keep runtime deps only.
- resple/package.xml
  - Drop <depend>message_runtime</depend> (not a ROS 2 dependency); ensure only runtime deps for used messages.
- resple/src/RESPLE.cpp and Mapping.cpp
  - Create QoS profiles with rclcpp::SensorDataQoS for all point cloud subs; small KeepLast for pubs.
  - Expose NUM_OF_THREAD, leaf sizes, and queue depths as parameters; read once at startup.
  - Switch main to rclcpp_components + MultiThreadedExecutor with callback groups; add composed launch.
  - Consider loaned messages for pub_cur_scan and Mapping's pub_global_map.

## Launch and packaging
- Add resple_composed.launch.py that loads components into component_container_mt with use_intra_process_comms enabled.
- Ensure Docker/containers use ROS 2 Jazzy base images and apt repos; cache colcon build to speed CI.

## Acceptance criteria
- ≤30% reduction in end-to-end latency (scan → pose) on representative dataset.
- ≤25% CPU reduction for the main process at steady-state.
- Zero allocator churn spikes under load (confirmed via tracing) and stable memory footprint.
- Clean build on Jazzy with default compiler, no custom arch flags.

## Next steps
1) Implement Phase 1 CMake + package.xml cleanups.
2) Add QoS and switch to MultiThreadedExecutor.
3) Add components + composed launch; enable intra-process.
4) Introduce loaned messages and buffer reuse in publishers.
5) Add tracing hooks and a perf baseline; iterate.
