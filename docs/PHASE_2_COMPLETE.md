# Phase 2 Complete: Executor, QoS, and Composition

**Date**: 2025-11-06  
**Status**: ✅ Complete and Tested

## Overview

Phase 2 successfully modernized RESPLE to use ROS 2 best practices including:
- Component-based architecture for composition
- Explicit QoS profiles
- Callback groups for concurrency separation
- MultiThreadedExecutor integration

## Changes Implemented

### 1. Made RESPLE Composable (`rclcpp::Node` Inheritance)

**Before:**
```cpp
class RESPLE {
public:
    RESPLE(rclcpp::Node::SharedPtr& nh) { ... }
};
```

**After:**
```cpp
class RESPLE : public rclcpp::Node {
public:
    explicit RESPLE(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("RESPLE", options) { ... }
};
RCLCPP_COMPONENTS_REGISTER_NODE(RESPLE)
```

**Benefits:**
- Can be loaded into a component container
- Enables intra-process communication (zero-copy)
- Better integration with ROS 2 lifecycle

### 2. Added Explicit QoS Profiles

**Before:**
```cpp
sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_type, 2000000, callback);
```

**After:**
```cpp
auto imu_qos = rclcpp::SensorDataQoS().keep_last(200).best_effort();
sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_type, imu_qos, callback, sensor_sub_opt);
```

**QoS Configuration:**
- **Sensors (IMU/LiDAR)**: `SensorDataQoS()` with BestEffort, KeepLast(100-200)
- **Critical data (`/est_window`, `/pose`)**: `QoS(50).reliable()`
- **Visualization (`/current_scan`)**: `QoS(2).best_effort()`

### 3. Added Callback Groups

```cpp
sensor_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
control_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

rclcpp::SubscriptionOptions sensor_sub_opt;
sensor_sub_opt.callback_group = sensor_cb_group;
```

**Benefits:**
- Sensor callbacks don't block control/estimation callbacks
- Better parallelism with MultiThreadedExecutor
- Prevents callback starvation

### 4. Integrated MultiThreadedExecutor

**Before:**
```cpp
rclcpp::Rate rate(200);
std::thread opt{&RESPLE::processData, &resple};
while (rclcpp::ok()) {
    rclcpp::spin_some(nh);
    rate.sleep();
}
```

**After:**
```cpp
std::thread opt{&RESPLE::processData, node.get()};

rclcpp::executors::MultiThreadedExecutor exec;
exec.add_node(node);
exec.spin();
```

**Benefits:**
- Proper thread lifecycle management
- Better load balancing across CPU cores
- Integrates with ROS 2 executor framework

### 5. Updated CMakeLists.txt for Component Registration

```cmake
find_package(rclcpp_components REQUIRED)

# Register components for composition
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "RESPLE"
  EXECUTABLE RESPLE_node
)
```

### 6. Fixed `shared_from_this()` Issues

**Problem:** `std::bad_weak_ptr` exception when calling `shared_from_this()` in constructor

**Solution:** Added overloaded utility functions accepting `Node&` instead of `Node::SharedPtr&`

```cpp
// Added to CommonUtils and LidarConfig
template <typename T>
static T readParam(rclcpp::Node& n, std::string name) { ... }

LidarConfig(rclcpp::Node& nh, const std::string& prefix) { ... }
```

**In constructor:**
```cpp
// Use *this instead of shared_from_this()
readParameters();  // Internally uses *this
LidarConfig lidar(*this, "");
```

## Files Modified

### Core Changes
- **`resple/src/RESPLE.cpp`**: Made composable, added QoS, callback groups, executor
- **`resple/src/Mapping.cpp`**: Updated constructor signatures for callback groups
- **`resple/include/utils/common_utils.h`**: Added `Node&` overloads
- **`resple/CMakeLists.txt`**: Added component registration
- **`resple/package.xml`**: Added `rclcpp_components` dependency

## Build and Test Results

### Build Status
```bash
$ colcon build --symlink-install --packages-select resple
Starting >>> resple
Finished <<< resple [1min 24s]

Summary: 1 package finished [1min 24s]
  1 package had stderr output: resple  # Only Eigen warnings
```

### Runtime Test
```bash
$ ros2 run resple RESPLE --ros-args --params-file config_helmdyn01.yaml
# ✅ No bad_weak_ptr errors
# ✅ Node starts successfully
# ✅ Subscriptions active
```

```bash
$ ros2 launch resple resple_helmdyn01.launch.py
# ✅ Launch file works
# ✅ RESPLE and Mapping nodes start
# ✅ RViz2 visualizes correctly
```

## Success Criteria

✅ **RESPLE runs with MultiThreadedExecutor** (no manual threads for callbacks)  
✅ **All topics use explicit QoS profiles**  
✅ **Component registration successful** (can be composed)  
✅ **Callback groups separate sensor/control execution**  
✅ **No `bad_weak_ptr` exceptions**  
✅ **Backward compatible** (existing launch files work)

## Next Steps (Phase 3)

- [ ] Use loaned messages for large PointCloud2 publishing
- [ ] Pre-allocate and reuse buffers (reduce allocations)
- [ ] Parameterize `NUM_OF_THREAD` and `NUM_MATCH_POINTS`
- [ ] Minimize PCL↔ROS conversions

## Performance Notes

- **Build time**: ~1min 24s (consistent with Phase 1)
- **Warnings**: Only Eigen uninitialized variable warnings (false positives)
- **Runtime**: Normal operation, no crashes or exceptions

## Technical Debt Addressed

1. ✅ Replaced integer queue sizes with semantic QoS profiles
2. ✅ Eliminated `shared_from_this()` constructor anti-pattern
3. ✅ Added proper callback group separation
4. ✅ Integrated with ROS 2 executor framework

## Known Limitations

- `processData()` still runs in a manual thread (will be converted to timer in future phase)
- Intra-process communication not yet enabled (requires composition container usage)
- Not yet using loaned messages for zero-copy publishing

---

**Document Version**: 1.0  
**Last Updated**: 2025-11-06  
**Next Phase**: Phase 3 - Zero-Copy and Data-Path Reductions
