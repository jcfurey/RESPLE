# Frame ID Quick Reference

## Default Configuration
```yaml
odom_frame_id: odom
body_frame_id: base_link
footprint_frame_id: base_footprint
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `odom_frame_id` | `odom` | Fixed world frame for odometry |
| `body_frame_id` | `base_link` | Robot body center frame |
| `footprint_frame_id` | `base_footprint` | Ground-projected frame for navigation |

## TF Tree Published

```
odom
└── base_link
    ├── base_footprint (if different from base_link)
    ├── imu
    └── lidar
```

## Common Configurations

### ROS2 Navigation (Nav2)
```yaml
odom_frame_id: odom
body_frame_id: base_link
footprint_frame_id: base_footprint
```

### Multi-Robot (with namespace)
```yaml
odom_frame_id: robot1/odom
body_frame_id: robot1/base_link
footprint_frame_id: robot1/base_footprint
```

### Aerial Vehicle (no footprint)
```yaml
odom_frame_id: odom
body_frame_id: base_link
footprint_frame_id: base_link  # Same as body_frame_id
```

### Custom Naming
```yaml
odom_frame_id: world
body_frame_id: sensor_platform
footprint_frame_id: sensor_platform_footprint
```

## Quick Test

```bash
# Check your TF tree
ros2 run tf2_tools view_frames

# View live transforms
ros2 topic echo /tf --no-arr

# Check if frames exist
ros2 run tf2_ros tf2_echo odom base_link

# Visualize in RViz2
ros2 run rviz2 rviz2
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Missing transforms | Check both RESPLE and Mapping have same config |
| Wrong frame names | Frame IDs are case-sensitive |
| TF lookup fails | Use `ros2 run tf2_tools view_frames` to debug |
| Footprint not needed | Set `footprint_frame_id: base_link` |

## Adding to Your Config

Add these lines at the top of your YAML config file:

```yaml
/**:
  ros__parameters:
    # Frame IDs for TF tree
    odom_frame_id: odom
    body_frame_id: base_link
    footprint_frame_id: base_footprint
    
    # ... rest of your config
```

## No Changes Needed For

✅ Launch files - they automatically pass config parameters  
✅ Existing configs - defaults match old hardcoded values  
✅ Code - frame IDs are read at startup

## More Information

See `docs/frame_id_configuration.md` for detailed documentation.
