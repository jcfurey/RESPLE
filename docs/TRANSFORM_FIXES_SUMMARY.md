# Transform Fixes Summary

## Overview
This document summarizes all the transform handling improvements applied to RESPLE based on your previous working implementation.

## Changes Made

### 1. RESPLE.cpp - TF2 Transform Support

#### Added Includes
```cpp
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
```

#### Added Member Variables
```cpp
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
bool have_imu_transform_ = false;
bool have_lidar_transform_ = false;
geometry_msgs::msg::TransformStamped lidar_to_baselink_;
geometry_msgs::msg::TransformStamped imu_to_baselink_;
```

#### IMU Transform Handling
- **New `transformImu()` function**: Properly transforms IMU data from IMU frame to base_link
  - Transforms orientation quaternion
  - Transforms angular velocity  
  - Transforms linear acceleration with centripetal/Coriolis correction
  - Formula: `a_transformed = R * a_raw + ω × (ω × (-r))`
  
- **Updated `getImuCallback()`**:
  - Looks up transform from IMU frame to `body_frame_id` on first call
  - Applies transform to all subsequent IMU messages
  - Throttled warnings if transform not available
  - Falls back to pass-through if no transform available

#### LiDAR Transform Handling
- **Updated `ousterLidarCallback()`**:
  - Looks up transform from LiDAR frame to `body_frame_id` on first call
  - Uses `tf2::doTransform()` to transform entire point cloud
  - Throttled warnings if transform not available
  - Falls back to pass-through if no transform available

### 2. Mapping.cpp - Transform Tree Structure

#### Added Includes
```cpp
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

#### Fixed Transform Tree
**Old (Incorrect) Structure:**
```
odom
└── base_link
    ├── base_footprint (child of base_link - WRONG!)
    ├── imu
    └── lidar
```

**New (Correct) Structure:**
```
odom
└── base_footprint (ground-projected, yaw-only)
    └── base_link (full 6-DoF with height + roll/pitch)
        ├── imu
        └── lidar
```

#### Transform Publishing Logic

**When `body_frame_id != footprint_frame_id`:**

1. **`odom → base_footprint`** (Ground-projected navigation frame):
   - Translation: `(x, y, 0)` - projected to ground plane
   - Rotation: Yaw-only quaternion (roll=0, pitch=0)
   - Extracted using: `yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))`

2. **`base_footprint → base_link`** (Elevation and attitude):
   - Translation: `(0, 0, z)` - height above ground
   - Rotation: Roll/pitch component only (yaw removed)
   - Computed as: `q_rollpitch = q_yaw^(-1) * q_full`

3. **`base_link → imu`**: Identity transform (sensors at base_link)
4. **`base_link → lidar`**: Identity transform (sensors at base_link)

**When `body_frame_id == footprint_frame_id`:**
- Single transform: `odom → base_link` with full 6-DoF pose

#### Odometry Message
- Changed `child_frame_id` from `body_frame_id` to `footprint_frame_id`
- This ensures Nav2 and other navigation tools reference the ground-projected frame

## Benefits

### 1. Multi-Sensor Integration
- ✅ Automatically handles sensors in different frames
- ✅ No need to manually specify extrinsics in config
- ✅ Works with existing TF tree (robot_state_publisher, URDF, etc.)

### 2. Navigation Compatibility  
- ✅ Provides proper `base_footprint` for 2D navigation (Nav2)
- ✅ Separates ground-projected pose from 3D body pose
- ✅ Yaw-only footprint allows easy 2D path planning

### 3. Proper Physics
- ✅ IMU acceleration properly accounts for sensor offset
- ✅ Centripetal acceleration correction: `ω × (ω × r)`
- ✅ Point clouds transformed to common reference frame

### 4. Robustness
- ✅ Graceful handling of missing transforms
- ✅ Throttled warnings prevent log spam
- ✅ Falls back to pass-through when transforms unavailable

## Configuration

### Frame ID Parameters (in YAML config):
```yaml
frame_id: base_link                    # Legacy, use body_frame_id
odom_frame_id: odom                    # Odometry reference frame
body_frame_id: base_link               # Main robot body frame
footprint_frame_id: base_footprint     # Ground projection for navigation
```

### TF Tree Requirements

RESPLE now expects these transforms to be published by your robot's `robot_state_publisher` or static transform publisher:

1. **`base_link → lidar_frame`** (or specific frame like `lidar0/lidar_frame`)
   - Published by: robot_state_publisher from URDF
   - Used by: Ouster callback

2. **`base_link → imu_frame`** (or specific frame like `lidar0/imu_frame`)
   - Published by: robot_state_publisher from URDF  
   - Used by: IMU callback

### Example Static Transform Publishers

If you don't have a URDF, you can publish static transforms:

```bash
# LiDAR transform (example)
ros2 run tf2_ros static_transform_publisher \
  0.05 0 -0.055 0 0 0 base_link lidar0/lidar_frame

# IMU transform (example)
ros2 run tf2_ros static_transform_publisher \
  0.05 0 -0.055 0 0 0 base_link lidar0/imu_frame
```

## Testing

### 1. Verify Transform Tree
```bash
# Generate PDF of transform tree
ros2 run tf2_tools view_frames

# View the PDF
evince frames.pdf
```

Expected tree:
```
odom
└── base_footprint
    └── base_link
        ├── imu
        ├── lidar
        └── [other sensors]
```

### 2. Check Transform Chain
```bash
# Test odom -> base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint

# Test base_footprint -> base_link  
ros2 run tf2_ros tf2_echo base_footprint base_link

# Test base_link -> lidar
ros2 run tf2_ros tf2_echo base_link lidar0/lidar_frame
```

### 3. Monitor Transforms
```bash
# Watch all transforms
ros2 topic echo /tf --no-arr

# Watch specific transform
ros2 topic echo /tf | grep base_footprint
```

### 4. Verify in RViz2
1. Launch RViz2
2. Set Fixed Frame to `odom`
3. Add TF display
4. Enable "Show Names" and "Show Axes"
5. Verify frame positions and orientations

## Troubleshooting

### "Waiting for LiDAR transform" warnings
- **Cause**: LiDAR frame not published in TF tree
- **Solution**: Check that robot_state_publisher is running or add static transform

### "Waiting for IMU transform" warnings  
- **Cause**: IMU frame not published in TF tree
- **Solution**: Check that robot_state_publisher is running or add static transform

### Footprint not at ground level
- **Cause**: Odometry initialization not at z=0
- **Solution**: Verify initial pose or adjust ground reference

### Roll/pitch in footprint frame
- **Cause**: Bug in yaw extraction (should not happen with current code)
- **Solution**: Check that `footprint_frame_id != body_frame_id`

### Nav2 planning failures
- **Cause**: Odometry child_frame doesn't match expected frame
- **Solution**: Ensure `footprint_frame_id` matches Nav2's `robot_base_frame` parameter

## Migration from Old Code

If you have existing configurations, no changes needed! The new code:
- ✅ Maintains backward compatibility
- ✅ Works with or without TF transforms
- ✅ Falls back to pass-through if transforms unavailable
- ✅ Uses same parameter names

## Related Files

- `RESPLE/resple/src/RESPLE.cpp` - Main estimation node
- `RESPLE/resple/src/Mapping.cpp` - Mapping and TF broadcasting
- `RESPLE/resple/config/config_*.yaml` - Configuration files
- `RESPLE/docs/frame_id_configuration.md` - Frame ID documentation
- `RESPLE/docs/CHANGELOG_frame_ids.md` - Change log

## References

- [ROS REP 105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [TF2 Migration Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Nav2 Configuration](https://navigation.ros.org/setup_guides/index.html)
