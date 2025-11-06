# Frame ID Configuration and TF Tree Management

## Overview

RESPLE now supports configurable frame IDs to enable better integration with ROS2 navigation stacks and multi-robot systems. This allows you to customize the TF tree structure to match your robot's conventions.

## Configuration Parameters

Add these parameters to your YAML configuration file (e.g., `config_helmdyn01.yaml`):

```yaml
/**:
  ros__parameters:
    # Frame IDs for TF tree
    frame_id: base_link                    # Deprecated, use body_frame_id instead
    odom_frame_id: odom                    # Odometry reference frame
    body_frame_id: base_link               # Main body/robot frame
    footprint_frame_id: base_footprint     # Ground projection frame for navigation
```

### Parameter Descriptions

- **`odom_frame_id`** (default: `"odom"`): The odometry reference frame. This is the parent frame for the robot's pose estimation.

- **`body_frame_id`** (default: `"base_link"`): The main robot body frame. This represents the physical center of the robot where sensors are mounted.

- **`footprint_frame_id`** (default: `"base_footprint"`): A ground-projected frame used by navigation stacks. This frame is automatically published as a child of `body_frame_id` with its Z coordinate projected to the ground plane (Z=0 in the odom frame).

- **`frame_id`** (deprecated): Legacy parameter for backward compatibility. Use `body_frame_id` instead.

## TF Tree Structure

The system publishes the following TF transforms:

```
odom
└── body_frame_id (e.g., base_link)
    ├── footprint_frame_id (e.g., base_footprint) [optional, if different from body_frame_id]
    ├── imu
    └── lidar
```

### Transform Details

1. **`odom` → `body_frame_id`**: Full 6-DoF pose from the spline-based estimator
   - Translation: 3D position (x, y, z)
   - Rotation: Full orientation quaternion

2. **`body_frame_id` → `footprint_frame_id`**: Ground projection transform
   - Only published if `body_frame_id != footprint_frame_id`
   - Translation: (0, 0, -z) where z is the body's height above ground
   - Rotation: Identity (no rotation)
   - This enables 2D navigation planning on the ground plane

3. **`body_frame_id` → `imu`**: IMU sensor frame
   - Currently identity transform (assumes IMU co-located with body frame)
   - Can be extended to use IMU extrinsics if needed

4. **`body_frame_id` → `lidar`**: LiDAR sensor frame
   - Currently identity transform
   - LiDAR extrinsics are handled internally via `q_lb` and `t_lb` parameters

## Use Cases

### Standard ROS2 Navigation

For Nav2 compatibility, use:

```yaml
odom_frame_id: odom
body_frame_id: base_link
footprint_frame_id: base_footprint
```

This publishes:
- `odom → base_link` (3D odometry)
- `base_link → base_footprint` (ground projection for 2D costmaps)

### Custom Robot Frames

For robots with custom naming conventions:

```yaml
odom_frame_id: robot/odom
body_frame_id: robot/base_link
footprint_frame_id: robot/base_footprint
```

### Multi-Robot Systems

Namespace your frames to avoid conflicts:

```yaml
odom_frame_id: robot1/odom
body_frame_id: robot1/base_link
footprint_frame_id: robot1/base_footprint
```

### Aerial or Underwater Vehicles

For vehicles that don't need footprint projection:

```yaml
odom_frame_id: odom
body_frame_id: base_link
footprint_frame_id: base_link  # Same as body_frame_id to disable projection
```

## Implementation Notes

### RESPLE Node

The RESPLE node reads frame parameters and uses them for:
- Publishing pose messages with correct `frame_id`
- Publishing point clouds with correct `frame_id`
- Logging frame configuration on startup

### Mapping Node

The Mapping node handles TF broadcasting:
- Reads the same frame parameters for consistency
- Publishes odometry messages with correct parent/child frames
- Broadcasts all TF transforms in the tree
- Handles conditional footprint transform based on configuration

### Backward Compatibility

The `frame_id` parameter is still supported but deprecated. If you have existing configurations:

```yaml
frame_id: base_link  # Old parameter
```

It will be treated as `body_frame_id`. However, we recommend migrating to the new explicit parameters:

```yaml
body_frame_id: base_link
footprint_frame_id: base_footprint
```

## Visualization in RViz2

To visualize the TF tree in RViz2:

1. Add a **TF** display
2. Set **Fixed Frame** to `odom` (or your configured `odom_frame_id`)
3. Enable **Show Names** to see frame labels
4. Enable **Show Axes** to see frame orientations

You should see the complete transform chain from `odom` to all sensor frames.

## Troubleshooting

### Missing Transforms

If you see TF lookup errors in navigation or other nodes:

1. Check that frame names match exactly (case-sensitive)
2. Verify parameters are set in both RESPLE and Mapping nodes
3. Use `ros2 run tf2_tools view_frames` to visualize your TF tree
4. Check timestamps with `ros2 topic echo /tf --no-arr`

### Incorrect Ground Projection

If the footprint frame doesn't align with the ground:

1. Verify your odometry initialization sets Z=0 at ground level
2. Check that the footprint transform calculation uses the correct sign
3. For aerial vehicles, consider setting `footprint_frame_id: base_link` to disable projection

### Multi-LiDAR Systems

For multi-LiDAR setups, each LiDAR can have unique extrinsics (via `q_lb`/`t_lb`). The single `lidar` frame in the TF tree represents the primary LiDAR reference frame. Individual LiDAR transforms are handled internally.

## Example Launch Configuration

Update your launch files to pass frame parameters:

```python
# In resple_helmdyn01.launch.py
parameters=[{
    'frame_id': 'base_link',
    'odom_frame_id': 'odom',
    'body_frame_id': 'base_link',
    'footprint_frame_id': 'base_footprint',
    # ... other parameters
}]
```

Or reference them from your YAML config file (recommended):

```python
parameters=[config_file]  # All frame params in YAML
```

## Future Enhancements

Potential improvements for future versions:

1. **Per-LiDAR TF Publishing**: Publish individual transforms for each LiDAR in multi-LiDAR systems
2. **IMU Extrinsics**: Use IMU-to-body extrinsics from config for TF publishing
3. **Dynamic Reconfiguration**: Allow frame IDs to be changed at runtime
4. **Static Transform Publisher**: Separate static transforms (imu, lidar) from dynamic ones (odom→body)

## References

- [ROS REP 105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [ROS2 TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Nav2 Frame Configuration](https://navigation.ros.org/)
