# RESPLE ROS 2 Jazzy Implementation Roadmap

## Current Status (2025-11-06)

### ✅ Completed
- **Phase 1: Build System Modernization (100%)**
  - Modern CMake with target-based properties
  - Build options: ENABLE_OPENMP, ENABLE_NATIVE_ARCH
  - Library architecture (no source duplication)
  - Clean dependency exports
  - Build time: 1m 21s (38% improvement)

- **Phase 0: Baseline + Profiling (85%)**
  - WSL2/Ubuntu 24.04 + ROS 2 Jazzy environment
  - RelWithDebInfo build with debug symbols
  - Tracing tools installed (ros2-trace, tracetools)
  - Baseline measurement script ready
  - Remaining: Run baseline with HelmDyn01 bag

### 📋 Documentation Structure
```
docs/
├── PHASE_0_1_PROGRESS.md       - Original session progress
├── PHASE_0_UPDATE.md           - Phase 0 latest status
├── PHASE_1_COMPLETE.md         - Phase 1 completion summary
├── ros2_jazzy_optimization_plan.md - Master roadmap
└── IMPLEMENTATION_ROADMAP.md   - This file (implementation plan)

scripts/
└── phase0_baseline.sh          - Automated baseline measurement
```

---

## Phase 2: Executor, QoS, Composition (P0→P1)

### Overview
Replace manual threading with ROS 2 best practices and enable zero-copy intra-process communication.

### Tasks

#### 2.1 Replace Manual Threading with MultiThreadedExecutor
**Current architecture** (RESPLE.cpp):
- Lines 885-890: Manual `spin_some()` + worker thread
- Worker thread calls `processData()` in infinite loop

**Target architecture**:
```cpp
// main() becomes:
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2  // num_threads - tune based on workload
    );
    
    auto resple_node = std::make_shared<rclcpp::Node>("RESPLE");
    RESPLE resple(resple_node);
    
    executor.add_node(resple_node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
```

**Changes required**:
- `processData()` → Convert to timer callback
- Remove manual thread creation (line 886)
- Remove `spin_some()` + `rate.sleep()` loop (lines 887-889)
- Use callback groups to separate sensor/processing callbacks

**Benefits**:
- Proper thread lifecycle management
- Better load balancing
- Integrates with ROS 2 lifecycle

---

#### 2.2 Make Nodes Composable (rclcpp_components)
**Why**: Enable loading RESPLE + Mapping into single process for intra-process communication.

**Changes for RESPLE.cpp**:
```cpp
#include <rclcpp_components/register_node_macro.hpp>

class RESPLE : public rclcpp::Node {
public:
    explicit RESPLE(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("RESPLE", options)
    {
        readParameters();
        // ... rest of constructor
    }
    
private:
    // ... existing members
};

RCLCPP_COMPONENTS_REGISTER_NODE(RESPLE)
```

**Changes for Mapping.cpp**:
- Same pattern: inherit from `rclcpp::Node`
- Register with `RCLCPP_COMPONENTS_REGISTER_NODE(Mapping)`

**CMakeLists.txt additions**:
```cmake
find_package(rclcpp_components REQUIRED)

# Create component libraries
rclcpp_components_register_node(resple
  PLUGIN "RESPLE"
  EXECUTABLE RESPLE_node
)

rclcpp_components_register_node(resple
  PLUGIN "Mapping"  
  EXECUTABLE Mapping_node
)
```

**New launch file** (`resple_composed.launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='resple_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='resple',
                plugin='RESPLE',
                name='RESPLE',
                parameters=[config_yaml],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='resple',
                plugin='Mapping',
                name='Mapping',
                parameters=[config_yaml],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
    )
    return LaunchDescription([container])
```

**Benefits**:
- Zero-copy between RESPLE and Mapping
- Single process → easier profiling
- ~30-50% reduction in message overhead

---

#### 2.3 Add Explicit QoS Profiles
**Current**: Integer queue sizes (implicit QoS)
**Target**: Explicit profiles matching data characteristics

**Changes for sensor subscriptions**:
```cpp
// Replace:
// sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
//     imu_type, 2000000, callback);

// With:
auto qos = rclcpp::SensorDataQoS()
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .keep_last(10);

sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
    imu_type, qos, callback);
```

**QoS recommendations by topic**:
| Topic | QoS Profile | Rationale |
|-------|-------------|-----------|
| `/livox/imu` | SensorDataQoS, BestEffort, KeepLast(10) | High-frequency, OK to drop old samples |
| `/livox/lidar` | SensorDataQoS, BestEffort, KeepLast(5) | Large messages, OK to drop |
| `/pose` | Reliable, KeepLast(10) | Consumers need every pose |
| `/current_scan` | BestEffort, KeepLast(2) | Large, visualization only |
| `/est_window` | Reliable, KeepLast(50) | Critical estimation data |

**Files to modify**:
- `RESPLE.cpp`: Lines 42, 64, 67, 70, 73, 76, 79 (subscriptions)
- `RESPLE.cpp`: Lines 44-47 (publishers)
- `Mapping.cpp`: Lines 127, 178, 223, 268, 313, 366 (subscriptions)

---

### Phase 2 Implementation Plan

**Step 1**: Refactor main() to use MultiThreadedExecutor (1-2 hours)
- Create timer callback for `processData()`
- Remove manual threading
- Test with HelmDyn01 bag

**Step 2**: Add explicit QoS profiles (30 min)
- Define QoS constants at file scope
- Replace integer queue sizes
- Verify topic communication

**Step 3**: Make nodes composable (2-3 hours)
- Refactor to inherit from `rclcpp::Node`
- Register components in CMake
- Create composed launch file
- Test standalone vs composed

**Step 4**: Enable intra-process (30 min)
- Add `NodeOptions().use_intra_process_comms(true)`
- Benchmark with/without to measure improvement

**Total estimated time**: 4-6 hours

---

## Phase 3: Zero-Copy and Data-Path Reductions (P1)

### Overview
Eliminate unnecessary copies and allocations in the data path.

### Tasks

#### 3.1 Use Loaned Messages for Publishing
**Target publishers**:
- `pub_cur_scan` (RESPLE.cpp:623) - publishes large PointCloud2
- `pub_global_map` (Mapping.cpp:71) - publishes large PointCloud2

**Pattern**:
```cpp
// Instead of:
// sensor_msgs::msg::PointCloud2 msg;
// pcl::toROSMsg(*cloud, msg);
// pub->publish(msg);

// Use:
auto loaned_msg = pub->borrow_loaned_message();
pcl::toROSMsg(*cloud, loaned_msg.get());
pub->publish(std::move(loaned_msg));
```

**Benefit**: Avoids copy for DDS serialization; ~20-30% reduction in publish latency for large messages.

---

#### 3.2 Minimize PCL↔ROS Conversions
**Current hotspots** (from code analysis):
- `RESPLE.cpp:620`: `pcl::toROSMsg()` on every scan
- `Mapping.cpp:69`: `pcl::toROSMsg()` on every map publish

**Optimization**: Use `sensor_msgs::PointCloud2Iterator` to construct messages directly when possible.

---

#### 3.3 Pre-Allocate and Reuse Buffers
**Current allocations** (frequent):
- `pc_last_ds` (RESPLE.cpp:102, 217)
- `pt_meas` (RESPLE.cpp:236) - deque grows/shrinks
- `pc_world` (RESPLE.cpp:218) - vector reallocations

**Improvements**:
```cpp
// Add as class members (pre-allocated, reused):
pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_reusable_;
std::vector<PointData> pt_meas_buffer_;

// In processData():
pc_reusable_->clear();  // instead of creating new
pc_reusable_->reserve(expected_size);
```

---

#### 3.4 Parameterize Hardcoded Values
**Current hardcoded** (common_utils.h):
- `NUM_OF_THREAD = 5` (line 13)
- `NUM_MATCH_POINTS = 5` (line 14)

**Make parameters**:
```cpp
// In constructor:
NUM_OF_THREAD = declare_parameter<int>("num_threads", 
    std::thread::hardware_concurrency());
NUM_MATCH_POINTS = declare_parameter<int>("num_match_points", 5);
```

**Benefit**: Tune performance per platform without recompiling.

---

### Phase 3 Implementation Plan

**Step 1**: Add loaned message publishing (1 hour)
**Step 2**: Parameterize NUM_OF_THREAD/NUM_MATCH_POINTS (30 min)
**Step 3**: Buffer pre-allocation (1-2 hours)
**Step 4**: Profile and measure improvement (1 hour)

**Total estimated time**: 3.5-4.5 hours

---

## Phase 4: Algorithmic and Concurrency Tuning (P1)

### Overview
Optimize hot loops and parallelization strategies.

### Tasks

#### 4.1 Parallelize Additional Hotspots
**Candidates** (from code structure):
- `MappingBase::transformCloud()` (Mapping.cpp:95-108) - loop over points
  - Currently serial; can parallelize with `#pragma omp parallel for`
- `mapIncremental()` (RESPLE.cpp:841-875) - loop over features
  - Already has OpenMP at higher level (line 148), but inner loops could benefit

**Pattern**:
```cpp
#pragma omp parallel for schedule(dynamic) num_threads(NUM_OF_THREAD)
for (size_t i = 0; i < points.size(); i++) {
    // point transformation
}
```

---

#### 4.2 Adaptive Voxel Sizes
**Current**: Fixed `ds_lm_voxel`, `ds_scan_voxel`
**Idea**: Increase leaf size when callback queue grows (system under load)

**Heuristic**:
```cpp
if (lidar_data.pc_buff.size() > threshold) {
    ds_filter_body.setLeafSize(ds_scan_voxel * 1.5, ...);
}
```

---

#### 4.3 ikd-Tree Update Batching
**Current** (RESPLE.cpp:873-874):
- Two separate `ikdtree.Add_Points()` calls per update

**Optimization**:
- Merge `PointToAdd` and `PointNoNeedDownsample` into single batch
- Reduces tree rebalancing overhead

---

### Phase 4 Implementation Plan

**Step 1**: Profile to identify true hotspots (perf record/report) (1 hour)
**Step 2**: Parallelize identified loops (1-2 hours)
**Step 3**: Implement adaptive voxel sizing (1 hour)
**Step 4**: Benchmark improvements (1 hour)

**Total estimated time**: 4-5 hours

---

## Phase 5: Observability, Tests, CI (P1)

### Overview
Production-quality instrumentation and validation.

### Tasks

#### 5.1 Add ros2_tracing Spans
**Code additions**:
```cpp
#include <tracetools/tracetools.h>

// In processData():
TRACEPOINT(process_data_start, ...);
// ... processing
TRACEPOINT(process_data_end, ...);
```

**Analysis**:
```bash
ros2 trace --session-name resple_profile ~/traces
# Run workload
ros2 trace stop
babeltrace2 ~/traces
```

---

#### 5.2 Add Micro-Benchmarks
**Framework**: Google Benchmark or ament_add_gtest

**Example** (test/benchmark_association.cpp):
```cpp
#include <benchmark/benchmark.h>

static void BM_pointBodyToWorld(benchmark::State& state) {
    for (auto _ : state) {
        Association::pointBodyToWorld(...);
    }
}
BENCHMARK(BM_pointBodyToWorld);
```

---

#### 5.3 Add Linters
**CMakeLists.txt**:
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_clang_tidy REQUIRED)
  ament_clang_tidy(
    CONFIG_FILE ${CMAKE_SOURCE_DIR}/.clang-tidy
  )
endif()
```

**.clang-tidy**:
```yaml
Checks: 'performance-*,modernize-*'
```

---

#### 5.4 CI Pipeline
**GitHub Actions** (.github/workflows/ci.yml):
```yaml
name: ROS 2 Jazzy CI
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-24.04
    steps:
      - uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: jazzy
      - uses: ros-tooling/action-ros-ci@v0.3
        with:
          package-name: resple
```

---

### Phase 5 Implementation Plan

**Step 1**: Add tracing spans (2 hours)
**Step 2**: Create benchmarks (2 hours)
**Step 3**: Add linters (1 hour)
**Step 4**: Set up CI (2 hours)

**Total estimated time**: 7 hours

---

## Phase 6: Optional Arch-Specific Speedups (P2)

### Overview
Platform-specific optimizations for production deployments.

### Tasks

#### 6.1 Gated Native Arch Builds
**Already implemented**: `ENABLE_NATIVE_ARCH=OFF` by default
**Enhancement**: Add runtime CPU feature detection

**Pattern**:
```cpp
#ifdef __AVX2__
    // Use AVX2 codepath
#else
    // Use portable codepath
#endif
```

---

#### 6.2 Eigen Alignment Tuning
**Current**: Default Eigen alignment (16 bytes)
**Optimization**: Use 64-byte alignment for AVX-512

**CMake**:
```cmake
if(ENABLE_AVX512)
  target_compile_definitions(${PROJECT_NAME} 
    PRIVATE EIGEN_MAX_ALIGN_BYTES=64)
endif()
```

---

### Phase 6 Implementation Plan

**Step 1**: Add CPU feature detection (1 hour)
**Step 2**: Conditional AVX2/AVX512 paths (2-3 hours)
**Step 3**: Benchmark on target hardware (1 hour)

**Total estimated time**: 4-5 hours (optional, hardware-dependent)

---

## Summary Timeline

| Phase | Priority | Estimated Time | Dependencies |
|-------|----------|----------------|--------------|
| Phase 0 completion | P0 | 30 min | Run baseline script |
| Phase 2 | P0→P1 | 4-6 hours | Phase 0 baseline |
| Phase 3 | P1 | 3.5-4.5 hours | Phase 2 |
| Phase 4 | P1 | 4-5 hours | Phase 3 |
| Phase 5 | P1 | 7 hours | Phase 2-4 |
| Phase 6 | P2 | 4-5 hours (optional) | All above |

**Total core implementation**: ~19-22 hours of focused development
**With optional Phase 6**: ~23-27 hours

---

## Next Immediate Steps

1. **Run Phase 0 baseline** (30 min)
   ```bash
   cd ~/ros2_ws/src/RESPLE
   ./scripts/phase0_baseline.sh
   ```

2. **Commit current progress** (10 min)
   ```bash
   git add -A
   git commit -m "RESPLE Jazzy: Phase 0-1 complete + Phase 2-6 roadmap"
   ```

3. **Start Phase 2.1** (MultiThreadedExecutor refactor)
   - Create branch: `git checkout -b phase2-executor`
   - Begin RESPLE.cpp refactoring

---

## Success Criteria

### Phase 2
- [ ] RESPLE runs with MultiThreadedExecutor (no manual threads)
- [ ] All topics use explicit QoS profiles
- [ ] Composed launch works (`resple_composed.launch.py`)
- [ ] Intra-process enabled and verified

### Phase 3
- [ ] Loaned messages used for PointCloud2 publishing
- [ ] Hardcoded values parameterized
- [ ] Memory allocations reduced (measurable via profiling)

### Phase 4
- [ ] CPU usage ≤25% lower than baseline (measured)
- [ ] Latency ≤30% lower than baseline (measured)

### Phase 5
- [ ] Tracing captures complete execution profile
- [ ] Benchmarks run in CI
- [ ] Linters pass with no warnings

### Phase 6 (optional)
- [ ] Native builds 5-10% faster on target hardware
- [ ] Portable builds still work on all platforms

---

**Document Version**: 1.0  
**Last Updated**: 2025-11-06  
**Status**: Ready for Phase 2 implementation
