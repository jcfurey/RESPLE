# Changelog: Configurable Frame IDs and TF Tree Management

## Summary

Added support for configurable frame IDs to enable better integration with ROS2 navigation stacks, multi-robot systems, and custom robot configurations.

## Changes Made

### 1. Code Changes

#### RESPLE Node (`resple/src/RESPLE.cpp`)
- **Changed hardcoded frame IDs to configurable parameters**:
  - `frame_id` → now read from parameters (default: `"base_link"`)
  - `odom_id` → now read from parameters (default: `"odom"`)
  - Added `body_frame_id` parameter (default: `"base_link"`)
  - Added `footprint_frame_id` parameter (default: `"base_footprint"`)
- **Added parameter reading in `readParameters()`**:
  - Reads frame parameters with sensible defaults
  - Logs configured frame IDs on startup for debugging
- **Removed redundant hardcoded constants**:
  - Removed `baselink_frame` and `odom_frame` constants

#### Mapping Node (`resple/src/Mapping.cpp`)
- **Changed hardcoded frame IDs to configurable parameters**:
  - Same parameters as RESPLE node for consistency
- **Enhanced `pubOdom()` TF broadcasting**:
  - Publishes `odom → body_frame_id` transform (6-DoF pose)
  - Conditionally publishes `body_frame_id → footprint_frame_id` (ground projection)
  - Publishes `body_frame_id → imu` transform (identity)
  - Publishes `body_frame_id → lidar` transform (identity)
- **Ground projection logic**:
  - Only publishes footprint transform if `body_frame_id != footprint_frame_id`
  - Projects body frame to ground plane with Z offset calculation

#### MappingBase Template (`resple/src/Mapping.cpp`)
- **Changed hardcoded frame IDs to configurable members**:
  - Same frame ID members as parent Mapping node
  - Used consistently in all derived classes (OusterBuff, Mid70AviaBuff, etc.)

### 2. Configuration Changes

All configuration files in `resple/config/` have been updated:
- `config_helmdyn01.yaml`
- `config_rcampus.yaml`
- `config_tudorun01.yaml`
- `config_eee02.yaml`
- `config_ntu_day_01.yaml`
- `config_ouster.yaml`
- `config_heap_testsite_hoenggerberg.yaml`
- `config_jungfraujoch_tunnel_small.yaml`

**Added parameters** (with defaults):
```yaml
# Frame IDs for TF tree
frame_id: base_link              # Deprecated, use body_frame_id
odom_frame_id: odom              # Odometry reference frame
body_frame_id: base_link         # Main body/robot frame
footprint_frame_id: base_footprint  # Ground projection frame for navigation
```

### 3. Launch Files

**No changes required!** 

All launch files already pass the full config YAML to both RESPLE and Mapping nodes:
```python
parameters=[config_yaml_fusion]
```

This means both nodes automatically receive the new frame parameters.

### 4. Documentation

Created comprehensive documentation:
- **`docs/frame_id_configuration.md`**: Complete guide to using frame IDs
  - Configuration examples
  - TF tree structure explanation
  - Use cases (Nav2, multi-robot, custom frames)
  - Troubleshooting guide
  - RViz2 visualization instructions

## TF Tree Structure

### Before (Hardcoded)
```
odom
└── base_link
    ├── imu
    └── lidar
```

### After (Configurable)
```
odom_frame_id (default: "odom")
└── body_frame_id (default: "base_link")
    ├── footprint_frame_id (default: "base_footprint") [conditional]
    ├── imu
    └── lidar
```

## Backward Compatibility

✅ **Fully backward compatible**

- All parameters have sensible defaults matching the old hardcoded values
- Existing configurations without frame parameters will continue to work
- The `frame_id` parameter is still supported (deprecated) for legacy configs
- No changes required to existing launch files

## Use Cases Enabled

### 1. Standard ROS2 Navigation (Nav2)
```yaml
odom_frame_id: odom
body_frame_id: base_link
footprint_frame_id: base_footprint
```

### 2. Multi-Robot Systems
```yaml
odom_frame_id: robot1/odom
body_frame_id: robot1/base_link
footprint_frame_id: robot1/base_footprint
```

### 3. Custom Frame Naming
```yaml
odom_frame_id: world
body_frame_id: sensor_platform
footprint_frame_id: sensor_platform  # Disable projection
```

### 4. Aerial/Underwater Vehicles
Set `footprint_frame_id` equal to `body_frame_id` to disable ground projection.

## Testing Recommendations

1. **Verify TF tree structure**:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

2. **Check transform timestamps**:
   ```bash
   ros2 topic echo /tf --no-arr
   ```

3. **Visualize in RViz2**:
   - Add TF display
   - Set Fixed Frame to your `odom_frame_id`
   - Enable "Show Names" and "Show Axes"

4. **Test with existing datasets**:
   - Run any existing launch file
   - Should work without modification
   - Check logs for frame ID messages

5. **Test custom configuration**:
   - Modify a config file with custom frame names
   - Verify Nav2 compatibility (if using)
   - Confirm multi-robot namespace isolation (if using)

## Migration Guide

### For Existing Users
No action required! Your existing configurations will continue to work.

### For New Features
Add frame parameters to your config YAML:
```yaml
odom_frame_id: odom
body_frame_id: base_link
footprint_frame_id: base_footprint
```

### For Multi-Robot Deployments
Namespace your frames in each robot's config:
```yaml
# Robot 1
odom_frame_id: robot1/odom
body_frame_id: robot1/base_link
footprint_frame_id: robot1/base_footprint

# Robot 2
odom_frame_id: robot2/odom
body_frame_id: robot2/base_link
footprint_frame_id: robot2/base_footprint
```

## Future Enhancements

Potential improvements for consideration:
1. Per-LiDAR frame publishing for multi-LiDAR systems
2. IMU frame with actual extrinsics (currently identity)
3. LiDAR frame with extrinsics from q_lb/t_lb
4. Static transform publisher separation
5. Dynamic reconfigure support

## Files Modified

### Source Code
- `resple/src/RESPLE.cpp`
- `resple/src/Mapping.cpp`

### Configuration
- `resple/config/config_helmdyn01.yaml`
- `resple/config/config_rcampus.yaml`
- `resple/config/config_tudorun01.yaml`
- `resple/config/config_eee02.yaml`
- `resple/config/config_ntu_day_01.yaml`
- `resple/config/config_ouster.yaml`
- `resple/config/config_heap_testsite_hoenggerberg.yaml`
- `resple/config/config_jungfraujoch_tunnel_small.yaml`

### Documentation
- `docs/frame_id_configuration.md` (new)
- `docs/CHANGELOG_frame_ids.md` (this file)

## References

- [ROS REP 105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [ROS2 TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Nav2 Documentation](https://navigation.ros.org/)
