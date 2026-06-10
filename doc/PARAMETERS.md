# RESPLE Parameter & Topic Reference

Complete reference for the RESPLE and Mapping nodes' runtime interface. Every
parameter is optional — omitting it keeps the listed default, and the defaults
reproduce the estimator's historical behaviour. The robustness features added
by the hardening series ([`HARDENING.md`](../HARDENING.md)) are all opt-in or
behaviour-preserving by default.

Example configurations: [`resple/config/config_pointcloud2.yaml`](../resple/config/config_pointcloud2.yaml)
(generic template — works with any `sensor_msgs/PointCloud2`),
[`resple/config/config_ouster.yaml`](../resple/config/config_ouster.yaml),
[`resple/config/config_demonstrator.yaml`](../resple/config/config_demonstrator.yaml).
The dataset configs (`config_nc_short.yaml`, …) are kept at the values used
for the published benchmarks.

The live-sensor launch files (`resple_pointcloud2.launch.py`,
`resple_ouster.launch.py`, `resple_demonstrator.launch.py`) accept a
`config_file` argument so a copied/adapted YAML can be used without editing
the installed one:

```bash
ros2 launch resple resple_pointcloud2.launch.py config_file:=/path/to/my_config.yaml
```

## Published topics

### RESPLE node (estimator)

| Topic | Type | Rate | Content |
| --- | --- | --- | --- |
| `odom` | `nav_msgs/Odometry` | per scan | Estimated pose + twist; covariance from the IEKF posterior |
| `pose` | `geometry_msgs/PoseStamped` | per scan | Pose only |
| `pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | per scan | Pose + 6×6 covariance |
| `current_scan` | `sensor_msgs/PointCloud2` | per map update | Deskewed scan in world frame |
| `est_window` | `estimate_msgs/Estimate` | per knot growth | Spline window for the Mapping node (internal protocol) |
| `resple_diagnostics` | `estimate_msgs/Diagnostics` | ~20 Hz | Typed estimator health (below) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Aggregated string-keyed status (standard ROS tooling) |

TF: `odom → base_link` (configurable via `odom/publish_tf`, `odom/invert_tf`,
frame-ID parameters). With `nis_recovery_mode: hold`, odometry/pose/TF
publication is suspended while the filter is flagged DIVERGED.

### Mapping node (visualization / map keeping)

| Topic | Type | Content |
| --- | --- | --- |
| `odometry` | `nav_msgs/Odometry` | Pose interpolated from the received spline window |
| `traj_path` | `nav_msgs/Path` | Trajectory history (capped at 10 000 poses) |
| `active_control_points` | `sensor_msgs/PointCloud` | The 4 active B-spline knots |
| `global_map` | `sensor_msgs/PointCloud2` | Accumulated map cloud |

### `estimate_msgs/Diagnostics` (the `resple_diagnostics` topic)

Typed, directly plottable in Foxglove / PlotJuggler. One message per processed
worker frame. Fields: spline knots (retained + monotonic total), input-buffer
depths, IEKF numerical-failure count, NIS + dof + filter state (OK/WARN/
DIVERGED), recovery state (hold active, reset count), pose-covariance trace +
λ_min, the correspondence funnel (`candidates → passed_window →
passed_distance → passed_plane → used`), deskew out-of-range count, map size,
prune/drop counters, and per-stage frame timings (drain / IEKF / deskew /
total / async map update). See
[`estimate_msgs/msg/Diagnostics.msg`](../estimate_msgs/msg/Diagnostics.msg)
for the authoritative field list.

## Parameters

### Frames & TF

| Parameter | Default | Meaning |
| --- | --- | --- |
| `frame_id` | `base_link` | Robot body frame (odometry `child_frame_id`) |
| `odom/frame_id` | `odom` | Odometry frame |
| `map/frame_id` | `map` | Map frame (Mapping node publications) |
| `odom/publish_tf` | `true` | RESPLE broadcasts `odom → base_link` |
| `odom/invert_tf` | `false` | Invert the broadcast direction |
| `map/publish_tf`, `map/invert_tf` | `true`, `false` | Same, Mapping node |
| `cov_pose`, `cov_twist` | `[0.1 ×6]` | Diagonals for the Mapping node's odometry covariance |

### Sensors

| Parameter | Default | Meaning |
| --- | --- | --- |
| `topic_imu` | `imu` | IMU topic (LIO mode) |
| `acc_ratio` | `false` | Accelerometer reports g-units (Livox built-ins): scale by 9.81 |
| `lidars` | — | List of sensor names; each gets its own block (below) |
| `<name>/topic_lidar` | — | Point cloud topic |
| `<name>/lidar_type` | — | `Ouster`, `Hesai`, `Mid360Boxi`, `HAP360`, `Mid70Avia`, `AviaResple`, or **`PointCloud2`** (generic: field layout introspected at runtime — works with any driver) |
| `<name>/blind` | — | Drop points within this radius (m) of the sensor |
| `<name>/q_lb`, `<name>/t_lb` | — | LiDAR→body extrinsics (quaternion `w,x,y,z`; translation m) |
| `<name>/w_pt` | — | Per-point measurement weight |
| `<name>/time_field` | `""` (auto) | `PointCloud2` type only: per-point time field name override |
| `<name>/time_unit` | `auto` | `auto` \| `s` \| `ms` \| `us` \| `ns` |
| `<name>/intensity_field` | `""` (auto) | Intensity/reflectivity field override |
| `lidar_time_offset` | `0.0` | Constant offset (s) added to LiDAR stamps |
| `imu_init_num_samples` | `50` | Stationary samples for gravity alignment |
| `imu_init_max_variance` | `5.0` | Reject init if accel variance exceeds this (m/s²)² |

### Estimator core

| Parameter | Default | Meaning |
| --- | --- | --- |
| `if_lidar_only` | `false` | LO (`true`) vs LIO (`false`) |
| `knot_hz` | — | B-spline knot rate (Hz); 100 is the canonical value |
| `n_iter` | `1` | IEKF iterations per update |
| `num_points_upd` | `100` | Max LiDAR points per IEKF step |
| `num_match_points` | `5` | k for the k-NN plane fit |
| `nn_thresh` | — | Point-to-plane residual threshold in the IEKF (m) |
| `coeff_cov` | — | Measurement covariance scaling |
| `ds_scan_voxel`, `ds_lm_voxel` | — | Scan / local-map voxel downsampling (m) |
| `point_filter_num` | `1` | Keep every n-th point |
| `cube_len` | `1000.0` | FOV-cube local map size (m) |
| `num_threads` | `5` | OpenMP threads for k-NN / transforms |
| `cov_P0`, `cov_RCP_*`, `std_sys_*`, `cov_acc`, `cov_gyro`, `cov_ba`, `cov_bg` | — | Filter prior / process / measurement noise (see example configs) |

### Robustness & diagnostics (HARDENING series)

All behaviour-preserving or opt-in by default. Section references are to
[`HARDENING.md`](../HARDENING.md).

| Parameter | Default | Meaning |
| --- | --- | --- |
| `spline_prune_keep_knots` | `600` | §3.1 sliding-window knot pruning: knots kept in memory (≈6 s at `knot_hz` 100). Interpolation over the retained window is bit-identical to the unpruned spline. `0` disables (unbounded growth); values 1–99 clamp to 100. Read by **both** nodes. |
| `nn_max_sq_dist` | `5.0` | §3.2 correspondence gate: max squared distance (m²) of the k-th nearest neighbor; the k-NN search radius is its square root. |
| `plane_fit_thresh` | `0.1` | §3.2 plane-fit residual threshold (m): every neighbor must lie within this distance of the fitted plane. |
| `plane_min_cond_ratio` | `0.0` | §3.2 degeneracy guard (rank-revealing-QR pivot ratio): rejects collinear / rank-deficient neighbor patches. `0` = off. Enabling changes which correspondences feed the filter — benchmark against a known-good dataset first; watch the funnel counters. |
| `nis_window` | `32` | §3.3 NIS consistency window (IEKF cycles). |
| `nis_warn_ratio` | `2.0` | WARN when windowed NIS mean exceeds ratio × dof. |
| `nis_diverged_ratio` | `4.0` | DIVERGED threshold (same form). |
| `nis_breach_limit` | `3` | Consecutive breaching windows before DIVERGED. |
| `nis_recovery_mode` | `"off"` | §3.3 action on DIVERGED: `"off"` = log + diagnostics only; `"hold"` = suspend odometry/pose/TF until the window recovers to OK (downstream fusion coasts on its other sources); `"reset"` = reinflate the IEKF covariance to the configure-time prior, keeping the state. Quote the value — bare `off` is YAML for `false`. |
| `map_prune_radius` | `0.0` | §3.4 keep only map points within this distance (m) of the pose; floored at 2× the detection range so it cannot bite live measurements. `0` = cube-only behaviour (`cube_len`). The FOV cube alone never deletes while the robot stays inside it. |
| `max_scan_buffer` | `0` | §2.3 per-LiDAR raw-scan cap (scans, drop-oldest). `0` = unbounded. Engaging drops are counted in diagnostics. |
| `max_imu_staging` | `2000` | §2.3 IMU staging-buffer cap (samples, drop-oldest); `0` disables. |

**Tuning workflow:** leave the defaults; record `resple_diagnostics`; only
then adjust. The funnel counters localize correspondence losses (sparse map
vs degenerate patches vs association outliers), `Spline Knots (total)` and
the drop counters show memory pressure, and the NIS fields show filter
consistency before/after any change.
