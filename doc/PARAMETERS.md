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

The Mapping node is optional — a downstream consumer of `est_window` that the
odometry never depends on. Launch files start it only with `use_mapping:=true`
(default `false`); see [`MAPPING_NODE.md`](MAPPING_NODE.md).

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
| `map_insert_lag_knots` | `0` | §6.3 internal-map insertion lag (RESPLE node): body-frame points are staged and only deskewed + inserted into the ikd-Tree once their timestamps are this many knots behind the spline edge, so the **reference map the IEKF matches against** is built from final knot values instead of trailing-edge estimates (cf. SLICT's marginalization-time map admission, arXiv:2211.03900; retrospective map refinement, arXiv:2503.21293). Also delays `current_scan` (same released points). `0` = upstream insert-at-edge. Recommended trial value `8`; **off by default pending bag validation** — this changes the odometry feedback loop. |
| `map_insert_cov_gate_deg` | `0.0` | §6.3 covariance gate for the insertion lag (RESPLE node): when > 0, staged points are released **early** whenever the edge-pose orientation std (deg, from `getLastPoseCovariance`) is below this value — gentle motion keeps a fresh reference map, aggressive motion gets the full `map_insert_lag_knots` hold (VoxelMap's uncertainty-convergence criterion, arXiv:2109.07082). Only meaningful with `map_insert_lag_knots > 0`. `0` = pure fixed-knot lag. Trial value: start at `1.0` and compare against the diagnostics' NIS/orientation fields. |
| `knot_rotation_warn_rad` | `0.05` | §6.3 knot under-resolution diagnostic (RESPLE node): each knot's **final** `ort_delta` norm is the rotation the spline absorbs in one knot interval; values above this threshold mean `knot_hz` under-fits the motion (Coco-LIC/ATI-CTLO lesson, arXiv:2309.09808 / 2407.20619) and trailing-edge jitter is expected. Fires a throttled WARN + counts in `/diagnostics` (`Knot Rotation Max (rad)`, `Knot Rotation Warnings`). Default 0.05 rad/knot = 5 rad/s at `knot_hz` 100 — silent on rover/handheld motion, fires on HelmDyn-class spin. `0` disables. |
| `map/transform_tolerance` | `0.0` | amcl-style future-dating (s) added to the `map→odom` TF stamp (Mapping node). The TF is stamped at the lagged path tip, so exact-time lookups in the map frame at fresh sensor stamps fail with extrapolation errors; set this to ≳ the lag + lookup horizon if a downstream consumer needs them. `0` = stamp at the tip (latest-available lookups unaffected). |
| `map_deskew_lag_knots` | `8` | §6.3 map lag (Mapping node only): scans wait until the spline edge is this many knots past their end before deskew into `/global_map`, and the path tip feeding the `map→odom` TF is lagged the same way — so both use knots the estimator has **finished** refining instead of the under-observed trailing edge (azimuth smear). The default is the convergence horizon + margin: the IEKF updates only the last 4 RCPs and `est_window` resends only the last 5 knots, so every knot a scan touches is final once the edge is ≥6 knots past it — larger values buy latency, not accuracy. Costs `lag × dt` of map latency (80 ms at `knot_hz` 100, well inside a 0.5 s map budget); odometry (`/odom`, `current_scan`, `odom→base_link`) is unaffected. `0` = bleeding-edge (old behavior). |
| `nn_max_sq_dist` | `5.0` | §3.2 correspondence gate: max squared distance (m²) of the k-th nearest neighbor; the k-NN search radius is its square root. |
| `plane_fit_thresh` | `0.1` | §3.2 plane-fit residual threshold (m): every neighbor must lie within this distance of the fitted plane. |
| `plane_min_cond_ratio` | `0.0` | §3.2 degeneracy guard (rank-revealing-QR pivot ratio): rejects collinear / rank-deficient neighbor patches. `0` = off. Enabling changes which correspondences feed the filter — benchmark against a known-good dataset first; watch the funnel counters. |
| `robust_kernel` | `"none"` | §3.2 M-estimator on the point-to-plane residuals: `"none"` (legacy weighting, bit-exact), `"huber"`, or `"cauchy"`. The IRLS weight `w(zp)` scales each point's information in the IEKF, smoothly downweighting outliers; the legacy `pt_thresh`/`cov_thresh` accept-reject gate is kept — the kernel softens what survives it (adaptive kernels: arXiv:2004.14938). Off pending bag A/B against the funnel + localizability diagnostics. |
| `robust_kernel_delta` | `0.1` | §3.2 kernel scale (m, residual units): the soft threshold where the loss transitions from quadratic to robust. Matches `plane_fit_thresh` scale by default. |
| `loc_gate_trans_min_eig` | `0.0` | §3.2 degenerate-direction gate (X-ICP-style, translation only): when an eigenvalue of the per-point-normalized LiDAR constraint matrix `E_tt = (1/N) Σ n nᵀ` falls below the gate, the LiDAR columns of the Kalman gain are projected out of that world direction for the RCP position rows — the matcher can no longer pull the pose along an axis it cannot observe (featureless-tunnel map-lock); IMU rows and the spline prior keep full authority there, and Joseph form keeps the covariance honest (it keeps growing along the gated axis). Eigenvalues sum to 1; healthy geometry runs ≳0.08 min-eig, tunnel collapse is ≤1e-3 — recommended trial gate `0.02`. `0` = off (decision-gated; "Loc Gate" diagnostics report axes/updates). |
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

## Reproducing the original (`main`) behavior for A/B comparison

The Mapping node's map *path* is logic-equivalent to upstream `main` (same
scan gating, same per-scan `/global_map` publication, same per-point deskew
math); the accuracy-relevant differences live in the estimator's numeric
path and a handful of defaults. To run an A/B against the original behavior:

Runtime parameters:

| Set | Restores |
| --- | --- |
| `map_deskew_lag_knots: 0` | Bleeding-edge map deskew (original timing, original aggressive-motion smear) |
| `spline_prune_keep_knots: 0` | Unbounded spline retention (original memory behavior) |
| `num_threads: 1` | Serial k-NN / transforms (original FP evaluation order) |
| `num_match_points: 5` | Already the default (upstream constant) |
| `nis_recovery_mode: "off"` | Already the default |
| `max_scan_buffer: 0`, `max_imu_staging: 2000` | Already the defaults: unbounded scan buffer; `main` hardcoded the 2000-sample IMU staging cap |

Build flags (numeric path — parameters cannot toggle these):

| Set | Restores |
| --- | --- |
| `-DENABLE_EIGEN_BLAS=OFF` | Pure-Eigen matrix products (`main` never routed Eigen through BLAS; BLAS rounds/orders FP differently) |

Not reproducible by toggles: `main` compiled with
`-DEIGEN_INITIALIZE_MATRICES_BY_NAN` in all builds (now Debug-only) and ran
the IEKF + map insertion fully synchronously. For a definitive comparison,
run the same bag through an `origin/main` build and a parity-configured
current build, then compare `evo_ape` on `/odom` and render both
`/global_map` streams side by side (HARDENING §6.3).
