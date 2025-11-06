# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Overview

RESPLE (Recursive Spline Estimation for LiDAR-Based Odometry) is a B-spline-based recursive state estimation framework for 6-DoF dynamic motion estimation. It supports four odometry variants: LiDAR-only (LO), LiDAR-inertial (LIO), Multi-LiDAR (MLO), and Multi-LiDAR-inertial (MLIO).

**Technology Stack**: ROS2 Humble, C++17, CMake, Ubuntu 22.04

## Build System

### Dependencies Installation
```bash
sudo apt install libomp-dev libpcl-dev libeigen3-dev
sudo apt install ros-humble-pcl*
# Optional for GrandTour dataset:
sudo apt install ros-humble-rosbag2-storage-mcap
```

### Compilation
```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/ASIG-X/RESPLE.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select estimate_msgs livox_ros_driver livox_interfaces livox_ros_driver2 resple
```

**Note**: The build must be done in the specified order due to package dependencies. Always use `--recursive` when cloning to include submodules (ikd-Tree).

### Docker Build
```bash
cd ~/path/to/src/RESPLE
docker build --ssh default --tag resple .
```

## Development Workflow

### Running the System

All launch commands require sourcing the ROS2 workspace first:
```bash
source install/setup.bash
```

Launch files are located in `resple/launch/` and follow the naming pattern `resple_<dataset>.launch.py`. Each launch file starts three nodes:
- **rviz2**: Visualization
- **RESPLE**: Main estimation algorithm
- **Mapping**: Map management

Example launch commands:
```bash
# HelmDyn dataset (Livox Mid360)
ros2 launch resple resple_helmdyn01.launch.py

# R-Campus dataset (Livox Avia)
ros2 launch resple resple_r_campus.launch.py

# TudoRun dataset (Livox Mid360)
ros2 launch resple resple_tudorun01.launch.py

# NTU VIRAL dataset (Ouster OS1-16)
ros2 launch resple resple_eee_02.launch.py

# MCD dataset (Livox Mid70)
ros2 launch resple resple_ntu_day_01.launch.py

# GrandTour dataset (Hesai XT32, Livox Mid360)
ros2 launch resple resple_heap_testsite_hoenggerberg.launch.py
```

In a separate terminal, play rosbag data:
```bash
source install/setup.bash
ros2 bag play /path/to/bag/
```

### Configuration Files

Configuration files are in `resple/config/` and follow the naming pattern `config_<dataset>.yaml`. Key configuration parameters:

- `if_lidar_only`: Set to `true` for LO mode, `false` for LIO mode
- `knot_hz`: B-spline knot frequency (typically 100 Hz)
- `ds_scan_voxel`: Downsampling voxel size for input scans
- `nn_thresh`: Nearest neighbor distance threshold
- `lidar_type`: Supported types include `HAP360`, `Mid360Boxi`, `AviaResple`, `Mid70Avia`, `Ouster`, `Hesai`
- `q_lb` and `t_lb`: LiDAR-to-body extrinsic calibration (rotation quaternion and translation)

### Docker Workflow

Allow docker graphics:
```bash
xhost +local:docker
```

Run container with mounted data and source:
```bash
docker run -it -e DISPLAY=$DISPLAY \
  -v .:/root/ros2_ws/src/RESPLE \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v ~/data/resple_dataset/:/root/data/resple_dataset \
  -v ~/data/grand_tour_box/datasets:/root/data/grand_tour_box/datasets \
  --name resple resple
```

Recompile inside container (if needed):
```bash
colcon build --packages-up-to resple
```

Manage container:
```bash
docker exec -it resple bash          # Attach second terminal
docker rm resple                      # Remove container
docker start resple                   # Start existing container
docker attach resple                  # Attach to running container
docker stop resple                    # Stop container
```

## Architecture

### Package Structure

- **resple/**: Main estimation package
  - `src/RESPLE.cpp`: Main ROS2 node handling sensor data and estimation loop
  - `src/Mapping.cpp`: Map management node
  - `include/Estimator.h`: Core recursive estimation algorithms (IEKF updates)
  - `include/SplineState.h`: B-spline state representation and interpolation
  - `include/Association.h`: Point-to-plane correspondence finding
  - `include/ikd-Tree/`: Incremental k-d tree for efficient nearest neighbor search
  - `include/utils/`: Common utilities (Eigen helpers, math tools)
  - `launch/`: ROS2 launch files for different datasets
  - `config/`: YAML configuration files

- **estimate_msgs/**: Custom ROS2 message definitions for spline states and estimates
- **livox_ros_driver/**, **livox_ros_driver2/**, **livox_interfaces/**: Livox LiDAR drivers
- **AviaResple_msgs/**, **HAP360_msgs/**, **Mid70Avia_msgs/**: LiDAR-specific message definitions

### Core Components

**RESPLE Node** (`RESPLE.cpp`): Main estimation node that:
- Subscribes to LiDAR (multiple types) and IMU topics
- Buffers and processes sensor data
- Manages the estimation loop calling `Estimator` methods
- Publishes estimates, poses, and point clouds

**Estimator** (`Estimator.h`): Template class implementing recursive estimation with two variants:
- `Estimator<24>`: LiDAR-only (24-state: 4 control points × 6 DoF)
- `Estimator<30>`: LiDAR-inertial (24-state + 3 accel bias + 3 gyro bias)
- Methods: `propRCP()` for propagation, `updateIEKFLiDAR()` and `updateIEKFLiDARInertial()` for measurement updates

**SplineState** (`SplineState.h`): B-spline representation of the trajectory:
- Maintains control points (knots) for position and orientation
- Provides interpolation methods with Jacobians for any timestamp
- Uses SO(3) left-perturbation for orientation representation

**Association** (`Association.h`): Data association between LiDAR points and map:
- Transforms points from body frame to world frame using spline interpolation
- Finds point-to-plane correspondences using ikd-tree
- Validates correspondences based on plane fitting quality

### Key Design Patterns

1. **Recursive Control Points (RCP)**: The estimator maintains a sliding window of 4 B-spline control points, propagating forward by predicting new control points and marginalizing old ones.

2. **Multi-LiDAR Support**: The system uses a map-based architecture (`lidars` and `lidars_data`) allowing multiple LiDAR configurations with different extrinsics.

3. **Iterated Extended Kalman Filter (IEKF)**: The update steps iterate until convergence (controlled by `max_iter` and `eps` parameters) to handle nonlinearities.

4. **OpenMP Parallelization**: Point processing and Jacobian computation are parallelized (controlled by `NUM_OF_THREAD`).

## File Modification Guidelines

### Adding New LiDAR Types
1. Add message definition package if needed (follow pattern of `HAP360_msgs/`)
2. Add callback in `RESPLE.cpp` (search for existing callbacks like `livoxLidarCallback`)
3. Add subscription logic in constructor based on `lidar_type` string
4. Create corresponding config file in `resple/config/`

### Modifying Estimation Parameters
- Edit `Estimator.h` to change state sizes or covariance initialization
- Adjust measurement noise models via config YAML files (`cov_acc`, `cov_gyro`, `coeff_cov`)
- Modify process noise via `cov_sys` and related parameters in YAML

### Creating New Launch Files
Follow the pattern in `resple/launch/resple_helmdyn01.launch.py`:
1. Reference the appropriate config YAML file
2. Launch three nodes: rviz2, RESPLE, and Mapping
3. Set log levels appropriately (typically 'warn' for reduced verbosity)

## Datasets

Official datasets available at: https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l (Password: RESPLE2025)

- **HelmDyn**: Livox Mid360 on helmet, dynamic motions (walking/running/jumping), ground truth from Qualisys motion capture
- **R-Campus**: Livox Avia on wheeled bipedal robot, large-scale campus environment
- **TudoRun**: Livox Mid360 on Unitree Go2 quadruped, indoor sequences with partial motion capture ground truth
- **NTU VIRAL**: Ouster OS1-16 on aerial platform
- **MCD**: Livox Mid70 on various platforms
- **GrandTour**: Hesai XT32 + Livox Mid360 on legged robot

Each dataset requires its corresponding launch file and config file.

## Important Notes

- The repository uses Git submodules (ikd-Tree). Always clone with `--recursive` flag.
- All executables are built to `install/lib/resple/`.
- ROS2 parameters are loaded from YAML files, not set via command line typically.
- The system requires proper time synchronization between sensors (handled via ROS2 timestamps).
- When switching between LO and LIO modes, only change `if_lidar_only` in the config file.
