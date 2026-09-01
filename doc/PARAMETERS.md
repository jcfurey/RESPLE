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

Field evaluation: [Exyn rotating-head tunnel LIO study
(2026-09-01)](FINDINGS_2026-09-01_exyn_rotating_head_tunnel.md).

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
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Mapping health (2026-07-11): replica knots, est_window queue depth/drops, scan funnel + cap drops, TF-guard verdicts; WARN on drops, ERROR on an active TF conflict |

### `estimate_msgs/Diagnostics` (the `resple_diagnostics` topic)

Typed, directly plottable in Foxglove / PlotJuggler. One message per processed
worker frame. Fields: spline knots (retained + monotonic total), input-buffer
depths, IEKF numerical-failure count, NIS + dof + filter state (OK/WARN/
DIVERGED), recovery state (hold active, reset count), pose-covariance trace +
λ_min, the correspondence funnel (`candidates → passed_window →
passed_distance → passed_plane → used`), deskew out-of-range count, map size,
prune/drop counters, per-stage frame timings (drain / IEKF / deskew /
total / async map update), the overload block (`rt_factor`, `backlog_ms`,
`latency_wall_ms`, `shed_scans`, `cycle_overruns`, `gap_extrap_knots` — see
"Resource-limited machines" below), and the TF ownership guard
(`tf_foreign_same_pair`, `tf_foreign_other_parent`, `tf_conflict_active`,
`tf_yielding` — see "TF ownership" below). See
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
| `odom/dense_pub_hz` | `0.0` (off) | Publish-density **floor** for pose/odom/TF (2026-07-10 dliio-lessons review). Steady state already publishes once per knot (`knot_hz`, ~100 Hz — measured on R_Campus baseline), so this changes nothing there. It back-fills, by spline interpolation at data-time stamps, the holes left when the edge advances by more than one knot per publish — `propRCP` jumping a scan gap, overload shedding (hazard 69), an NIS-hold release — exactly where downstream `lookupTransform` would otherwise extrapolate; capped at 1 s per back-fill. Values above `knot_hz` upsample the chain sub-knot (the spline is continuous); clamped to ≤ 1 kHz. `0` = pre-feature behaviour. |
| `map/publish_tf`, `map/invert_tf` | `true`, `false` | Same, Mapping node |
| `cov_pose`, `cov_twist` | `[0.2, 0.2, 0.2, 0.1, 0.1, 0.1]` | Diagonals for the Mapping node's odometry covariance (position/linear first three, orientation/angular last three) |
| `tf_extrinsics` | `true` | `true`: the `base_link ← sensor` TF carries the mounting extrinsic (production convention; YAML `q_lb`/`t_lb` is an *extra* offset, normally identity). `false`: no TF consulted; clouds/IMU stay in their native frames and YAML `q_lb`/`t_lb` is the single extrinsic (upstream/dataset-replay convention). **Set per-rig in each config YAML** (next to `q_lb`/`t_lb`), not in the launch — most published-dataset configs ship `false` (YAML extrinsic), while the production/template configs and any bag whose recording carries its own `/tf_static` sensor mounting (e.g. the ERDC `config_06042026`/`config_07052026` sets) ship `true`. Never publish a TF that duplicates a non-identity YAML extrinsic: the two compose and cancel. |
| `tf_wait_timeout` | `10.0` | Seconds to wait for the sensor TF before falling back to the YAML-only convention (was: scans dropped forever) |
| `publish_extrinsic_tf` | `false` | When the YAML `q_lb`/`t_lb` convention is in effect (`tf_extrinsics: false`, or the `tf_wait_timeout` fallback fired), latch `frame_id → <cloud header frame>` **once on `/tf_static`** with exactly the `T_base←sensor` the estimator applies — so the TF tree is complete without an external `static_transform_publisher` (which has leaked stale extrinsics across replay runs before). LiDAR frames always; the IMU frame too **when `q_ib`/`t_ib` is configured** (2026-07-11 — the mounting is then known, unlike the identity-guess case). Skipped with a WARN if the sensor frame already exists in the TF tree (a bag's own `/tf_static`, a URDF) — prefix RESPLE's frames in that case, see `config_07052026.yaml`. |
| `tf_conflict_action` | `"warn"` | TF ownership guard (both nodes, shared name): what to do when **another node** publishes the TF pair this node broadcasts (RESPLE: `odom → frame_id`; Mapping: `map → odom`; post-inversion). `"warn"` = throttled ERROR + diagnostics, transforms unchanged. `"yield"` = keep publishing odometry/path **topics** but suspend our own TF broadcast while the foreign publisher is active — resumes automatically once it goes quiet for `tf_conflict_quiet_sec`. Self-published transforms are recognized by exact stamp match, so the guard never trips on its own DDS loopback. Also detects the watched **child** frame being claimed by a *different* parent (a TF child has exactly one parent — two parents break the tree even without a same-pair collision). See "TF ownership" below. |
| `tf_conflict_quiet_sec` | `5.0` | How long the foreign publisher must stay quiet before `yield` resumes our broadcast (also the window for the `tf_conflict_active` diagnostic). |
| `tf_absent_warn_sec` | `10.0` | Reverse misconfiguration check: when `publish_tf` is **false** (an external owner is expected — e.g. the odom EKF), WARN once if *nobody* has published the pair after this many seconds. `0` disables. |

### TF ownership — coexisting with an external localization stack

TF is a tree: every child frame has exactly **one** parent, and tf2 keeps the
latest transform per pair with no concept of ownership. Two nodes publishing
the same pair interleave at their two rates and every consumer sees the pose
flicker/jump between them; a child claimed by two different parents breaks
lookups outright. The guard above watches the raw `/tf` + `/tf_static` stream
for exactly these two failures on the pair each node broadcasts, and its
verdicts surface in `resple_diagnostics` (`tf_foreign_same_pair`,
`tf_foreign_other_parent`, `tf_conflict_active`, `tf_yielding`) and as
WARN/ERROR in `/diagnostics`.

Recommended wiring when RESPLE runs inside a larger stack
(`robot_localization` odom EKF, elevation_mapping, a global localizer):

- **Odometry fusion (robot_localization / odom EKF owns the odom TF).** Set
  `odom/publish_tf: false` — RESPLE stays a pure odometry *source*: the EKF
  fuses the `/odometry` topic (whose pose covariance is the live IEKF
  posterior, plus the §6.7 advisory inflation in degenerate geometry) and
  owns `odom → base_footprint`. Note the on-robot configs typically set
  `frame_id: base_footprint`, which makes RESPLE's would-be TF pair *exactly*
  the EKF's pair — with both `publish_tf` flags on they fight for the same
  transform, which is precisely the flicker the guard flags. The
  `tf_absent_warn_sec` check covers the opposite mistake (RESPLE demoted but
  the EKF never launched: TF consumers starve silently).
- **elevation_mapping and other TF consumers** publish no TF themselves —
  no conflict possible; they just need one consistent `odom → base` chain
  and RESPLE's pose covariance.
- **Map frame.** The Mapping node is visualization-side and owns
  `map → odom` (plus a one-shot identity latch on `/tf_static` for the
  cold-start window). If a global localizer / SLAM owns the map frame, set
  `map/publish_tf: false` or simply run without Mapping
  (`use_mapping:=false`, the launch default).
- **`tf_conflict_action: yield`** is the belt-and-braces option for rigs
  where the owner set may change at runtime (e.g. an EKF that starts late or
  restarts): whichever RESPLE/Mapping TF has a live foreign owner is
  suspended automatically instead of fighting, and resumes when the owner
  disappears. Default stays `warn` (detect-only).

Two-parent subtlety worth knowing: with `frame_id: base_link` and an EKF
publishing `odom → base_footprint`, nobody collides on a *pair*, but the URDF
static `base_footprint → base_link` plus RESPLE's `odom → base_link` give
`base_link` **two parents** — the tree is just as broken. The guard reports
this as a foreign-parent claim.

Two guard refinements (2026-07-11): a foreign transform arriving on
`/tf_static` is **sticky** — tf2 buffers keep static transforms forever, so
the conflict persists for every running consumer even if the publisher dies;
`yield` therefore holds until a lifecycle re-configure, and the pre-`/clock`
sim-time corner can no longer age the claim out of the freshness window. And
the absence check now **re-arms**: with `publish_tf: false`, an external
owner that published and then *died mid-run* triggers a fresh one-shot WARN
("owner stopped publishing"), distinct from the never-wired-up case; it
re-arms each time the owner resumes.

Worked example — `robot_localization` odom EKF fusing wheel odometry +
RESPLE (illustrative; tune the config vectors to your platform). RESPLE side:
`frame_id: base_footprint`, `odom/publish_tf: false` (see
`config_ouster.yaml`'s frame block). EKF side:

```yaml
ekf_odom:
  ros__parameters:
    world_frame: odom
    odom_frame: odom
    base_link_frame: base_footprint
    publish_tf: true                    # THE owner of odom->base_footprint

    odom0: /wheel/odometry              # wheel encoders: velocities only —
    odom0_config: [false, false, false, # wheel pose integrates slip; let the
                   false, false, false, # filter integrate the twist instead
                   true,  true,  false,
                   false, false, true,
                   false, false, false]

    odom1: /localization/resple/odom    # RESPLE (this package's `odom` topic,
    odom1_config: [true,  true,  true,  # remapped/namespaced per your launch):
                   true,  true,  true,  # full 6-DoF pose, absolute. Its
                   false, false, false, # covariance is the live IEKF posterior
                   false, false, false, # (plus the §6.7 tunnel inflation if
                   false, false, false] # enabled), so the EKF weighs it
                                        # against wheel slip honestly — do not
                                        # override it with a static matrix.
```

Fuse RESPLE as the single absolute pose source (as above) so the EKF's odom
frame tracks RESPLE's gauge; if you later add a second absolute pose source,
switch one of them to `_differential: true` to avoid origin fights. RESPLE's
twist block is also available (fuse velocities instead of pose) — its twist
covariance is likewise the real posterior since the 2026-07-02 fix.

Related bonus: once the stack's TF tree is up (EKF + URDF), RESPLE can take
its **initial attitude** from it instead of waiting for a stationary gravity
window — `init_attitude_source: tf` (or `tf_gravity_check`) with a
gravity-aligned `init_attitude_frame`; see the Sensors table above. Useful
when the robot starts on a slope or is already moving at launch.

### Sensors

| Parameter | Default | Meaning |
| --- | --- | --- |
| `topic_imu` | `imu` | IMU topic (LIO mode) |
| `acc_ratio` | `false` | Accelerometer reports g-units (Livox built-ins): scale by 9.81 |
| `sensor_qos_reliable` | `false` | QoS for both IMU and LiDAR subscriptions. `false` preserves the historical sensor-data best-effort policy. Set `true` only when every upstream sensor publisher offers reliable delivery; it prevents whole scans from being lost under executor/DDS load, but a reliable subscriber is incompatible with a best-effort-only publisher and will receive no data from it. |
| `lidars` | — | List of sensor names; each gets its own block (below) |
| `<name>/topic_lidar` | — | Point cloud topic |
| `<name>/lidar_type` | — | `Ouster`, `Hesai`, `Mid360Boxi`, `HAP360`, `Mid70Avia`, `AviaResple`, or **`PointCloud2`** (generic: field layout introspected at runtime — works with any driver) |
| `<name>/blind` | — | Drop points within this radius (m) of the sensor |
| `<name>/scan_line` | `0` | Number of scan lines/rings. **Only the three Livox `CustomMsg` callbacks read it** (`livoxLidarCallback`, `livoxLidar2Callback`, `livoxAVIACallback`), where it drives the per-line reordering. It is INERT for `Ouster`, `Hesai`, `Mid360Boxi` and the generic `PointCloud2` type — all 20 shipped configs set it regardless, which is harmless but misleading. Previously undocumented. |
| `<name>/q_lb`, `<name>/t_lb` | — | body→LiDAR extrinsics: `p_lidar = q_lb · p_body + t_lb` (quaternion `w,x,y,z`; translation m). The code inverts them (`q_bl = q_lb⁻¹`, `t_bl = q_lb⁻¹·(−t_lb)`) to map LiDAR points into the body frame — do **not** supply the LiDAR-pose-in-body here, it is the inverse of that |
| `<name>/w_pt` | `0.01` (code fallback) | Per-point LiDAR measurement **VARIANCE** in m², despite the name — it is used as `var_pt`, and `R⁻¹` for the row is `1/(var_pt·noise_scale)`. So SMALLER means the point is trusted MORE, the opposite of what "weight" suggests. `0.01` = σ 0.1 m; shipped configs use 0.01 (16 of 20), 0.05 (3) and 0.5 (1). A missing or mistyped entry falls back to 0.01 in code (it was `1e-9` = σ 31 µm, i.e. effectively infinite trust, until hazard 93). It also sets the scale of the `coeff_cov` escape clause and the `lidar_gate_sigma` gate, both of which are expressed relative to it. |
| `<name>/time_field` | `""` (auto) | `PointCloud2` type only: per-point time field name override |
| `<name>/time_unit` | `auto` | `auto` \| `s` \| `ms` \| `us` \| `ns` |
| `<name>/intensity_field` | `""` (auto) | Intensity/reflectivity field override |
| `lidar_time_offset` | `0.0` | Constant offset (s) **SUBTRACTED** from LiDAR stamps: every call site computes `stamp_ns - time_offset` (12 sites across both nodes; this row previously said "added", which is backwards). So a POSITIVE value moves LiDAR data EARLIER in time — use it when the LiDAR stamps lag the IMU/spline time base. Paired everywhere with a `stamp_ns < time_offset` early-return that drops messages whose stamp precedes the offset (the early-sim-time case). 5 shipped configs use 0.1. |
| `imu_init_num_samples` | `50` | Stationary samples for gravity alignment |
| `imu_init_max_variance` | `5.0` | Reject init if accel variance exceeds this (m/s²)² |
| `lo_imu_wait_timeout` | `10.0` | LO mode only: seconds to wait for IMU before initializing without gravity alignment (identity orientation). LIO always requires IMU |
| `init_attitude_source` | `gravity` | Initial `world←base_link` attitude source. `gravity` = accelerometer only (self-contained; **blocks until a stationary window exists**). `tf` = base_link attitude from the TF tree (`init_attitude_frame → frame_id`); works while moving, no stationary window needed. `tf_gravity_check` = TF authoritative, but the accelerometer cross-checks it whenever a stationary window is available |
| `init_attitude_frame` | `""` | The **gravity-aligned** reference frame for the TF modes (e.g. an ENU `map`, a levelled survey frame, or an AHRS output). Required for `tf`/`tf_gravity_check`; if unset those modes fall back to `gravity` with a warning. The world Z axis is defined along gravity, so this frame **must** be gravity-aligned or the map tilts off-gravity |
| `init_yaw_from_tf` | `false` | `false`: take only roll/pitch from the TF, keep yaw a free gauge (preserves RESPLE's *relative* odometry contract). `true`: also adopt the TF yaw (absolute heading — only if a downstream consumer needs RESPLE's odom yaw to match the attitude frame) |
| `gravity_magnitude` | `9.81` | Magnitude of the world-frame gravity constant (m/s²). Lower slightly for high altitude, or set to `1.0` if the IMU reports accel in g-units alongside `acc_ratio` |
| `init_attitude_consistency_deg` | `3.0` | `tf_gravity_check`: roll/pitch disagreement (deg) between TF and gravity above which a WARN fires and the diagnostic goes WARN. A wrong `imu→base_link` extrinsic or a non-level attitude frame shows up here — the delta ≈ the mounting/frame error. Surfaced as *Init Attitude TF-vs-Gravity Delta (deg)* in diagnostics |
| `range_ref` | `3.0` | Close-range down-weighting reference (m): point variance is inflated by `(range_ref/r)²` for `r < range_ref` so one nearby surface cannot dominate the update. By default this is **absolute** (each point judged only on its own range) — in a confined space where every return is close it therefore starves the whole update (9× at 1 m walls, 36× at 0.5 m) and the estimator coasts on the prior/IMU then snaps. Prefer `range_noise_relative: true` (below) over lowering/disabling it; `0` disables the mechanism entirely |
| `range_noise_relative` | `false` | Make the close-range inflation **per-scan relative**: divide every point's factor by the scan's own minimum (i.e. the factor of its farthest valid return). Fixes the confined-space starvation above — tunnel, culvert, pipe, container, truck bed — while leaving open scenes **bit-identical** (with any return at/beyond `range_ref` the normalizer is exactly 1, so anti-domination is preserved unchanged). Relative weighting *between* returns in a scan is untouched either way; only the absolute level is anchored. Cost is one min-reduction over the update's points (≤ `num_points_upd`), so it is free on the hot path. Recommended for any mission that can enter a confined space; default `false` keeps the legacy numerics pending bag A/B. See `utils/range_weighting.h`, `test/test_range_weighting.cpp` |
| `range_noise_scale_max` | `900.0` | Cap on that variance inflation (default = the old implicit `(3/0.1)²` ceiling) |
| `cov_bias_acc_rw`, `cov_bias_gyro_rw` | `cov_RCP_pos_new·cov_sys_pos`, `cov_RCP_ort_new·cov_sys_ort` | LIO bias random-walk variance per knot step (rows 24–29 of Q). Defaults reproduce the magnitude the bias block received before the 2026-07-02 Q-indexing fix |
| `q_ib`, `t_ib` | identity, `[0,0,0]` | **YAML IMU extrinsic** (2026-07-11): `p_imu = q_ib · p_body + t_ib`, the same direction convention as the per-lidar `q_lb`/`t_lb`. Only consulted in the YAML convention (`tf_extrinsics: false`) — TF mode gets the IMU mounting from the tree. When non-identity, IMU samples are rotated (with the lever-arm correction) through the same `transformImu` path TF mode uses, and gravity init applies the rotation too; previously a tilted IMU was **silently fused unrotated** in YAML mode. Identity default = the legacy pass-through, bit-identical. Also latched to `/tf_static` under `publish_extrinsic_tf` (the mounting is known here, unlike the identity-guess case) |

### Estimator core

| Parameter | Default | Meaning |
| --- | --- | --- |
| `if_lidar_only` | `false` | LO (`true`) vs LIO (`false`) |
| `knot_hz` | — | B-spline knot rate (Hz); 100 is the canonical value |
| `init_map_window_ms` | `100` | Stationary LiDAR history used to seed the first local map. RESPLE waits until the requested interval and the following complete-scan boundary are available, voxel-filters the combined seed at `ds_lm_voxel`, then re-anchors the filter at the first post-seed scan. A rotating sensor can set this to at least one head revolution plus one LiDAR scan so startup geometry does not depend on head phase. Values ≤0 fall back to 100; values above 60,000 clamp to 60,000. Keep the platform stationary through this interval. |
| `n_iter` | `1` | IEKF iterations per update |
| `num_points_upd` | `100` | Max LiDAR points per IEKF step |
| `num_match_points` | `5` | k for the k-NN plane fit. Declared with a **hard-fail** `ParameterDescriptor` range of 3–10: a value outside it makes the node REFUSE TO START (rclcpp rejects the parameter), it is not clamped. Note a CUDA build additionally caps the GPU path at 8 — 9 and 10 fall back to the ikd-Tree with a WARN. |
| `nn_thresh` | — | Point-to-plane residual threshold in the IEKF (m). **Read the `coeff_cov` row: this only binds when `coeff_cov <= 1`.** |
| `coeff_cov` | — | Escape hatch on the row accept test, which is `\|zp\| < nn_thresh` **\|\|** `lid_cov < var_pt*coeff_cov` with `lid_cov = H·P·Hᵗ + var_pt`. Since `lid_cov >= var_pt` always, any value `> 1` makes the second disjunct hold for a converged `P` — it short-circuits and **`nn_thresh` never binds**. `1.0` makes it unsatisfiable and thereby activates `nn_thresh` (what `config_narrow_tunnel.yaml` does). With `coeff_cov > 1` the only outlier rejection that fires is `findCorresp`'s `\|pd2\| < sqrt(range)/9`: 0.11 m at 1 m, 0.35 m at 10 m, 0.79 m at 50 m — at full weight unless `robust_kernel` is on. `cov_escape_admits` in the Diagnostics msg counts the rows this admits past `nn_thresh`. |
| `ds_scan_voxel`, `ds_lm_voxel` | — | Scan / local-map voxel downsampling (m) |
| `point_filter_num` | `1` | Keep every n-th point |
| `cube_len` | `1000.0` | FOV-cube local map size (m) |
| `num_threads` | `5` | OpenMP threads for k-NN / transforms. Declared with a **hard-fail** `ParameterDescriptor` range of 1–16: outside it the node refuses to start. Inside it, the value is then clamped at runtime to `max(1, hardware_concurrency − 2)` with a WARN — so "rejected" and "clamped" apply to different ranges, which the note below only described for the second. |
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
| `plane_min_cond_ratio` | `0.0` | §3.2 planarity guard: rejects collinear / rank-deficient neighbour patches. The test is `λ2/λ1` of the **centred** scatter `Σ(p−c)(p−c)ᵀ` — the ratio of the two in-plane extents, normalized to `(0,1]`. `0` = off. **Semantics changed 2026-07-27.** It used to be a rank-revealing-QR pivot ratio on the *uncentred* `Σ p pᵀ`, which is dominated by the centroid outer product, so its verdict tracked distance from the odom origin (falling ~5 decades from 1 m to 200 m) instead of planarity — and at the one value any shipped config used (`0.05`) it rejected **100% of patches at every range**. Measured on the fixed test with `num_match_points: 5`, good-patch / noisy-collinear acceptance, flat across 1–200 m: `0.05` → 90%/55%, `0.10` → 82%/24%, `0.20` → 62%/8%, `0.35` → 37%/2%. Separation is intrinsically weak at 5 neighbours; prefer `num_match_points >= 8` if you rely on this guard. Watch the funnel counters after enabling. |
| `lidar_gate_sigma` | `0.0` | **Adaptive (Mahalanobis) outlier gate** on point-to-plane rows: drop a row when `zp² > sigma²·lid_cov`, i.e. when the residual is beyond `sigma` standard deviations of the *predicted innovation* rather than beyond a fixed distance. This is the mechanism the `coeff_cov` row explains is missing: with `coeff_cov > 1` nothing at the update stage rejects an outlier, and `findCorresp`'s absolute test admits 0.35–0.79 m of off-plane error in the mid/far field — where a bad correspondence has the longest lever arm on rotation. Being normalized, one setting covers both regimes: it relaxes automatically while `P` is large (init, post-gap fast-forward) and tightens as the filter converges, where an absolute `nn_thresh` must be loose enough for the worst case at all times. With the shipped `var_pt = 0.01` (σ 0.1 m), `sigma: 5.0` sits at ~0.5 m converged — the `nn_thresh` the configs already specify. `0` = off (default, bit-identical legacy path), per the §3.2 rule that outlier-rejection changes wait on bag A/B; enable it in `config_narrow_tunnel.yaml` style. See `utils/lidar_gate.h`, `test/test_lidar_gate.cpp` |
| `lidar_gate_max_reject_frac` | `0.5` | Escape hatch for the gate above, and the reason it cannot make things worse. A normalized gate trusts `P`; if `P` is wrong-and-small while the pose is genuinely off, the gate would reject exactly the measurements that fix it and the error latches (the classic gating pathology). So when more than this fraction of an update's candidate rows would be rejected, that is read as evidence against the **prior**, the gate disarms for that update and every row is admitted — i.e. legacy behaviour. The gate can therefore never remove more than this fraction of any one update's rows. `1.0` disables the escape (not recommended), `0.0` disables the gate. Watch `lidar_gate_disarms`: a steady stream means the filter is over-confident (cross-check `nis`), not that the scans are bad |
| `robust_kernel` | `"none"` | §3.2 M-estimator on the point-to-plane residuals: `"none"` (legacy weighting, bit-exact), `"huber"`, or `"cauchy"`. The IRLS weight `w(zp)` scales each point's information in the IEKF, smoothly downweighting outliers; the legacy `pt_thresh`/`cov_thresh` accept-reject gate is kept — the kernel softens what survives it (adaptive kernels: arXiv:2004.14938). Off pending bag A/B against the funnel + localizability diagnostics. |
| `robust_kernel_delta` | `0.1` | §3.2 kernel scale (m, residual units): the soft threshold where the loss transitions from quadratic to robust. Matches `plane_fit_thresh` scale by default. |
| `loc_gate_trans_min_eig` | `0.0` | §3.2 degenerate-direction gate (X-ICP-style, translation only): when an eigenvalue of the per-point-normalized LiDAR constraint matrix `E_tt = (1/N) Σ n nᵀ` falls below the gate, the LiDAR columns of the Kalman gain are projected out of that world direction for the RCP position rows — the matcher can no longer pull the pose along an axis it cannot observe (featureless-tunnel map-lock); IMU rows and the spline prior keep full authority there, and Joseph form keeps the covariance honest (it keeps growing along the gated axis). Eigenvalues sum to 1; healthy geometry runs ≳0.08 min-eig, tunnel collapse is ≤1e-3 — recommended trial gate `0.02` (validated `0.10` is safest: 0/3 catastrophes, no healthy regression). `0` = off (decision-gated; "Loc Gate" diagnostics report axes/updates). |
| `loc_gate_cov_rate` | `0.0` | §6.7 publish-side advisory covariance inflation (pairs with the gate). The raw IEKF posterior only reports cm-scale *local* uncertainty along a gated axis, so a downstream EKF over-trusts RESPLE in a tunnel. When `>0`, a once-per-frame leaky integrator grows an inflation `Σ` (m²) added to the **published** `/odom` pose-covariance translation block along persistently severely-degenerate directions — advisory only; the internal filter, gain and trajectory are byte-identical (APE unchanged). Steady-state along-axis variance ≈ `rate/(1−decay)`. Validated 2026-06-12: tunnel along-axis std 3.7 cm → ~30 cm (cond# → 70+) while smooth ground motion (R_Campus) stays at baseline (~1 cm); aggressive *handheld* motion (HelmDyn) can false-positive (~12 cm) — not the wheeled-deployment profile. `0` = off (decision-gated; "Loc Gate Cov Infl" diagnostic reports the max std added). |
| `loc_gate_cov_decay` | `0.99` | §6.7 per-frame leak for the inflation integrator; with `rate` sets the steady-state inflation `rate/(1−decay)` (e.g. `0.01/0.99` → 1.0 m² ⇒ 1 m std) and the relaxation time constant when the axis becomes observable again. |
| `loc_gate_cov_persist` | `5` | §6.7 consecutive severely-degenerate frames required before the inflation starts accumulating — hysteresis against transient degeneracy. (On the wheeled platform `5` suffices; raise it only if smooth-motion validation shows spurious inflation.) |
| `loc_gate_cov_min_eig` | `0.0` | §6.7 eigenvalue threshold below which a direction counts as *severely* degenerate for inflation — deliberately stricter than the projection gate `loc_gate_trans_min_eig` (which must stay lenient to catch degeneracy early). Contributions are deficit-weighted `(thresh−λ)/thresh`, so only genuine collapse inflates. `0` = derive as `loc_gate_trans_min_eig / 10`. |
| `nis_window` | `32` | §3.3 NIS consistency window (IEKF cycles). |
| `nis_warn_ratio` | `2.0` | WARN when windowed NIS mean exceeds ratio × dof. |
| `nis_diverged_ratio` | `4.0` | DIVERGED threshold (same form). |
| `nis_breach_limit` | `3` | Consecutive breaching windows before DIVERGED. |
| `nis_recovery_mode` | `"off"` | §3.3 action on DIVERGED: `"off"` = log + diagnostics only; `"hold"` = suspend odometry/pose/TF until the window recovers to OK (downstream fusion coasts on its other sources); `"reset"` = reinflate the IEKF covariance to `nis_reset_cov` (a large recovery covariance), keeping the state. Quote the value — bare `off` is YAML for `false`. |
| `nis_reset_cov` | `1.0` | §3.3 the uniform covariance (variance, m²/rad²) the `"reset"` recovery reinflates the IEKF state covariance to. Must be **large**: divergence is an over-confident (too-small) covariance, so the recovery target has to dominate it for new measurements to re-engage. A small value deflates the filter further and freezes it on the diverged state — the pre-fix bug, which reset to the ~2e-6 startup prior. Only consulted when `nis_recovery_mode: "reset"`. |
| `map_prune_radius` | `0.0` | §3.4 keep only map points within this distance (m) of the pose; floored at 2× the detection range so it cannot bite live measurements. `0` = cube-only behaviour (`cube_len`). The FOV cube alone never deletes while the robot stays inside it. |
| `max_scan_buffer` | `0` | §2.3 per-LiDAR raw-scan cap (scans, drop-oldest). `0` = unbounded. Engaging drops are counted in diagnostics. |
| `max_imu_staging` | `2000` | §2.3 IMU staging-buffer cap (samples, drop-oldest); `0` disables. |
| `max_latency_ms` | `0` | Overload hardening ("drop scans, stay real-time"): per-LiDAR **data-time** latency budget. When a scan arrives, queued scans of the *same* LiDAR whose stamps are more than this many ms older than it are shed before the worker ever sees them (counted separately as `shed_scans`; throttled WARN while engaging). Data-time semantics: independent of bag playback rate, needs no wall clock, survives sim-time jumps; a bag-loop restart (time going backwards) never sheds. The newest queued scan is only shed when the incoming push immediately replaces it. `0` = off (legacy unbounded latency). |
| `gap_extrap_decay` | `1.0` | Overload hardening: damping for `propRCP`'s constant-velocity extrapolation across data gaps (after sheds/drops). `1.0` = off — the literal legacy expression, bit-identical. `<1`: for each knot beyond `gap_extrap_free_knots` within one propRCP call, the extrapolated velocity is scaled by `decay^(excess)`, converging to a hold-at-newest — the excursion across a gap is bounded by ~`free + v/(1−decay)` knots of travel instead of growing linearly, so the first post-gap update yanks the pose far less (and the IMU residual gate is not tripped by runaway extrapolation). Covariance propagation is unchanged (keeps growing across the gap). |
| `gap_extrap_free_knots` | `3` | Knots per `propRCP` call exempt from the decay. Steady-state calls add ≤ ~2 knots, so the default engages the damping only on genuine gaps. Inert while `gap_extrap_decay` = 1 (the `gap_extrap_knots` diagnostic still counts gap fast-forwards). |
| `executor_threads` | `0` | Cap on the `MultiThreadedExecutor` thread pool (both nodes). `0` = rclcpp default (one thread per core — oversubscription on small shared machines). The sensor callback group is MutuallyExclusive, so `2` is enough almost everywhere. |
| `publish_current_scan` | `true` | `false` skips the `/current_scan` world-cloud copy + publish (RESPLE node). Visualization-only output; odometry and the internal map are unaffected. |
| `publish_est_window` | `true` | `false` skips the `/est_window` publish (RESPLE node). **Only valid with the Mapping node off** (`use_mapping:=false`, the launch default) — `/est_window` is Mapping's sole input and it starves silently without it. A loud WARN fires at configure when disabled. |
| `map/max_scan_buffer` | `200` | Mapping node: per-sensor scan-buffer cap (was hardcoded 200 and dropped **silently**; now counted as `cap_dropped` in the 2 s summary + throttled WARN — cap-drops are permanent map holes). |
| `map/publish_min_interval_ms` | `0` | Mapping node: batch `/global_map` publishes — transformed scans accumulate and flush at most every N ms. **Batches, never drops**: each publish is an incremental scan viewers accumulate, so content is preserved, just in fewer/larger messages. `0` = per-scan (legacy). |

Additionally, `num_threads` (both nodes) is now clamped at runtime to
`max(1, hardware_concurrency − 2)` with a WARN — the shipped configs assume a
big machine, and the old value 8 oversubscribed 4–6-core boxes so badly the
worker lost its cores mid-IEKF. This is the one deliberate default-behaviour
change of the overload-hardening series; on ≥ `num_threads + 2` cores nothing
changes.

**Tuning workflow:** leave the defaults; record `resple_diagnostics`; only
then adjust. The funnel counters localize correspondence losses (sparse map
vs degenerate patches vs association outliers), `Spline Knots (total)` and
the drop counters show memory pressure, and the NIS fields show filter
consistency before/after any change.

### Resource-limited machines (odometry "jumping around")

On a small or shared CPU (4–8 cores next to other software) the default
profile fails in a specific shape: the worker falls behind the sensor stream,
the un-capped input queue grows, and the worker then drains the whole backlog
in one burst — the pose output freezes and then jumps. If scans get dropped,
`propRCP` extrapolates constant-velocity across the gap without a horizon and
the first post-gap update yanks the pose back (worse in LIO: a runaway
extrapolation makes the interpolated IMU prediction garbage, which trips the
IMU residual gate and disables the IMU exactly when it is the only
stabilizer).

Diagnose before tuning — the overload block of `resple_diagnostics` (and the
same metrics in `/diagnostics`):

| Symptom | Metric | Knob |
| --- | --- | --- |
| Output freezes then jumps; latency grows over a run | `backlog_ms` climbing, `rt_factor < 1` | `max_latency_ms: 200` — bounds the data-time backlog by shedding the *oldest* queued scans; the estimator stays at the front of the stream. `backlog_ms` then saw-tooths ≤ ~budget instead of growing. |
| Pose lurches right after WARN-logged sheds/drops | `shed_scans` / `dropped_scans` climbing + jump at the same time | `gap_extrap_decay: 0.9` — damps the across-gap extrapolation toward a hold so the re-entry correction is bounded. `gap_extrap_knots` counts how often gaps are being fast-forwarded. |
| Everything slow, other processes starved, no single hot spot | `cycle_overruns` climbing, whole machine loaded | `num_threads` (auto-clamped), `executor_threads: 2`, and the publisher gates `publish_current_scan: false` / `publish_est_window: false` (the latter only with Mapping off). |
| Map has holes / Mapping WARNs about dropped scans | `cap_dropped` in the Mapping 2 s summary | Mapping is optional — prefer running without it when starved. Otherwise raise `map/max_scan_buffer` and batch with `map/publish_min_interval_ms: 500` (batching never loses content). |

`config_low_resource.yaml` bundles the recommended values (budget 200 ms,
decay 0.9, 3 OpenMP + 2 executor threads, both publishers gated, Mapping
assumed off).

**Validation harness:** `scripts/overload_rehearsal.sh <bag>` runs the same
bag three times under a CPU constraint (baseline / `max_latency_ms` /
`+ gap_extrap_decay`) and prints the per-leg overload metrics side by side —
including the max pose step, the "jump" this hardening exists to remove. Run
it on the target machine with a representative bag; CI's bag-free
`e2e-smoke` job (`scripts/e2e_smoke.sh`) covers gross pipeline regressions
but not this closed-loop behaviour.

Caveats: `rt_factor` reads ≈1 *by design* once shedding is active (data time
is fast-forwarded) — read it together with `shed_scans`; `backlog_ms` is the
rate-independent ground truth. `latency_wall_ms` is only meaningful live or
with `ros2 bag play --clock` + `use_sim_time`. Multi-lidar: the budget is
per-LiDAR (a lagging-clock lidar is judged only against itself); after a
shed, the existing fast-forward + admission gate drop other lidars' stale
points — that is the chosen policy, not a bug.

### Clock domains — bag replay at throttled speeds

Both nodes keep three time domains strictly separated (2026-07-08 audit), so
**replay behaviour is invariant to `ros2 bag play --rate`** as long as you
play with `--clock` and run the nodes with `use_sim_time:=true`:

- **Data time** (message stamps): everything the estimator computes — spline
  knots, deskew, IEKF admission, the `max_latency_ms` shedding decision, the
  TF-guard self-stamp matching. Never touches any clock.
- **ROS time** (node clock = sim time on bags): every *wait-for-data*
  timeout and every data-facing window — `tf_wait_timeout` (both nodes),
  `lo_imu_wait_timeout`, the init-attitude TF wait, the TF-guard
  `tf_conflict_quiet_sec` freshness and `tf_absent_warn_sec` absence check,
  and Mapping's `map/publish_min_interval_ms` batch interval. At
  `--rate 0.25` a 10 s wait covers 10 s of *bag* time, so slow replay can no
  longer expire a TF/IMU wait before the bag delivers the data. All of these
  survive the two sim-time traps: the clock reading 0 before the first
  `/clock` message (windows start at the first valid sample, not at a bogus
  epoch delta) and backwards jumps on a bag loop restart (windows re-arm
  instead of wedging; `utils/sim_time_wait.h`, unit-tested).
- **Wall time** (monotonic clock): only things that genuinely measure the
  real machine — per-stage compute timings, `cycle_overruns` (the 50 ms
  worker budget), the wall denominator of `rt_factor` (its definition), the
  lifecycle bounded joins, and Mapping's 50 ms publish-I/O pacing (content
  is unaffected; each publish ships the full accumulated state).

Without `--clock`/`use_sim_time` the node clock is system time, and the ROS
time domain degrades to the pre-audit wall-clock behaviour — so **always
replay with `--clock` and `use_sim_time:=true`** when testing at non-1x
rates. (One known cosmetic residue: `RCLCPP_*_THROTTLE` suppression windows
use the node clock and may stay quiet briefly after a bag loop restart —
logging only, no behavioural effect.)

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
