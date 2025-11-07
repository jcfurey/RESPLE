# RESPLE Initialization Improvements

## Summary

Improved the initialization process to be more tolerant and prevent tilted maps when RESPLE starts while the robot is moving or settling.

## Changes Made

### 1. RESPLE.cpp Initialization Logic

**Previous Behavior:**
- Used only the first 15 IMU samples to estimate initial gravity vector
- No check for IMU stationarity
- Would initialize even if robot was moving, causing tilted maps

**New Behavior:**
- Waits for configurable number of IMU samples (default: 50)
- Computes variance of accelerometer readings
- Only initializes if variance is below threshold (stationary robot)
- Provides clear warning messages when waiting for initialization
- Logs successful initialization with sample count and variance

### 2. New Configuration Parameters

Add these parameters to your config YAML files (optional - defaults shown):

```yaml
# IMU Initialization Parameters (LIO mode only)
imu_init_num_samples: 50        # Number of IMU samples to average for gravity estimation
imu_init_max_variance: 5.0      # Maximum accelerometer variance (m²/s⁴) to consider robot stationary
```

**Parameter Guidelines:**
- `imu_init_num_samples`: 
  - Minimum: 10 (enforced)
  - Default: 50 (recommended for most cases)
  - Increase to 100+ for very noisy IMUs or to ensure longer settling time
  
- `imu_init_max_variance`:
  - Default: 5.0 works for most scenarios
  - Decrease (e.g., 2.0) if you need stricter stationarity requirements
  - Increase (e.g., 10.0) if robot has vibrations even when "stationary"
  - Set very high (e.g., 100.0) to effectively disable the check

### 3. Runtime Behavior

**During Initialization:**
```
[WARN] [1699999999.123456789] [RESPLE]: Waiting for 50 IMU samples for initialization (current: 23)
[WARN] [1699999999.456789012] [RESPLE]: IMU readings too noisy for initialization (variance: 8.2341 > 5.0000). Ensure robot is stationary or increase imu_init_max_variance parameter.
[INFO] [1699999999.789012345] [RESPLE]: IMU initialization successful (samples: 50, variance: 3.2156)
```

**Recommendations:**
1. Launch RESPLE at the same time as your robot stack - no need to wait manually
2. Let RESPLE wait for stable IMU readings automatically
3. If initialization takes too long, check:
   - Robot is actually stationary
   - IMU data is being published
   - Consider adjusting `imu_init_max_variance` if needed

### 4. Coordination with Mapping

The Mapping node automatically waits for RESPLE's initialization:
- Mapping subscribes to `/start_time` topic published by RESPLE
- Only begins processing after RESPLE successfully initializes
- No changes needed to Mapping.cpp

## Example Configuration

```yaml
/**:
  ros__parameters:
    # ... existing parameters ...
    
    # LiDAR-Inertial mode
    if_lidar_only: false
    
    # IMU initialization (only used when if_lidar_only=false)
    imu_init_num_samples: 50      # Wait for 50 IMU samples (~1 second at 50 Hz)
    imu_init_max_variance: 5.0    # Allow moderate vibrations
    
    # ... rest of config ...
```

## Testing

To verify the improvements:

1. **Test with moving robot:**
   ```bash
   # Start robot stack (with IMU publishing)
   # Immediately launch RESPLE
   ros2 launch resple your_launch_file.launch.py
   
   # Watch for initialization messages
   # Should wait until robot settles before initializing
   ```

2. **Monitor initialization:**
   ```bash
   ros2 topic echo /start_time --once
   # Note: This will only output after successful initialization
   ```

3. **Adjust parameters if needed:**
   - If initialization is too strict: increase `imu_init_max_variance`
   - If initialization is too lenient: decrease `imu_init_max_variance` or increase `imu_init_num_samples`

## Notes

- These parameters only affect LIO mode (`if_lidar_only: false`)
- LO mode (LiDAR-only) is unaffected and initializes immediately
- The improvements prevent common issue of tilted initial map alignment
- Zero runtime overhead after initialization completes
