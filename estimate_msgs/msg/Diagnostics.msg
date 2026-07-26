# Consolidated RESPLE estimator diagnostics (HARDENING Phase 4).
#
# Published by the RESPLE node on the relative topic "resple_diagnostics"
# (=> /localization/resple/resple_diagnostics under the production namespace)
# once per processed worker frame (~20 Hz while data flows). Typed fields so
# Foxglove / PlotJuggler can plot them directly, unlike the string-keyed
# diagnostic_updater output on /diagnostics (which remains for aggregation).

std_msgs/Header header

# --- B-spline state ---
int64 knot_count            # retained knots (numKnots)
int64 knot_total            # monotonic total incl. pruned (totalKnots)

# --- input buffers ---
int64 lidar_buffer_depth    # sum of per-LiDAR pt_buff depths after drain
int64 imu_buffer_depth      # worker-side imu_buff depth after drain
int64 imu_staging_depth     # imu_int_buff depth (callback staging, pre-drain)

# --- IEKF health ---
uint64 iekf_numerical_failures   # cumulative, active estimator
float64 nis                      # last update's normalized innovation squared
int32 nis_dof                    # its degrees of freedom
uint8 filter_state               # resple::health::FilterState: 0 OK / 1 WARN / 2 DIVERGED
bool recovery_hold_active        # §3.3 'hold' gate currently suspending odom/TF
uint64 recovery_resets           # §3.3 'reset' covariance reinflations, cumulative

# --- pose covariance health (6x6 pose block at maxTimeNs) ---
float64 cov_trace
float64 cov_lambda_min

# --- correspondence funnel (§3.2, last IEKF update) ---
uint32 corresp_candidates
uint32 corresp_passed_window
uint32 corresp_passed_knn
uint32 corresp_passed_plane
uint32 corresp_used

# --- deskew / map ---
uint64 deskew_out_of_range  # cumulative out-of-spline-range deskew queries
int64 map_size              # ikd-tree node count
uint64 map_radius_prunes    # §3.4 radius-prune operations, cumulative

# --- bounded-buffer drops (§2.3, cumulative; non-zero = caps engaging) ---
uint64 dropped_scans
uint64 dropped_imu

# --- worker stage timings, last frame (ms) ---
float32 drain_ms            # buffer drain + measurement collection
float32 iekf_ms             # propRCP + updateIEKF*
float32 deskew_ms           # deskew transform + world-cloud accumulation
float32 frame_total_ms      # whole frame (drain excluded; matches Avg Computation Time)
float32 map_update_ms       # last completed async map update (incremental + FOV/radius prune)

# --- overload / real-time health (2026-07-07 hardening) ---
# rt_factor: spline data-time advance / wall time over a ~2 s window. <1 =
# falling behind the stream. NOTE: with latency shedding active this stays
# ~1 by design (data-time is fast-forwarded) — read it TOGETHER with
# shed_scans to distinguish "keeping up" from "keeping up by dropping".
float32 rt_factor
# backlog_ms: max over lidars of (newest arrived scan stamp - spline
# maxTimeNs), in data time. Rate-independent overload ground truth.
float32 backlog_ms
# latency_wall_ms: node clock now() - newest processed measurement stamp.
# Only meaningful live, or on bag replay with --clock + use_sim_time.
float32 latency_wall_ms
# shed_scans: cumulative scans dropped by the max_latency_ms budget
# (distinct from dropped_scans = max_scan_buffer count-cap drops).
uint64 shed_scans
# cycle_overruns: worker outer iterations exceeding the 50 ms budget.
uint32 cycle_overruns
# gap_extrap_knots: cumulative knots extrapolated beyond the free window
# (gap fast-forwards; damped when gap_extrap_decay < 1).
uint64 gap_extrap_knots

# --- TF ownership guard (2026-07-07) ---
# Foreign publishers observed on the TF pair this node broadcasts
# (post-inversion odom->frame_id; self is stamp-matched out). same_pair =
# another node fighting for the exact pair; other_parent = the child frame
# claimed by a different parent (broken tree). tf_conflict_active = a foreign
# observation within the last tf_conflict_quiet_sec; tf_yielding = the
# 'yield' action is currently suspending our own broadcast.
uint64 tf_foreign_same_pair
uint64 tf_foreign_other_parent
bool tf_conflict_active
bool tf_yielding

# --- LiDAR row gating (2026-07-26 accuracy pass) ---
# cov_escape_admits: cumulative rows admitted into an IEKF update by the
# `lid_cov < var_pt*coeff_cov` disjunct of the accept test while |zp| was NOT
# below nn_thresh -- i.e. rows the residual threshold would have rejected.
# Because lid_cov >= var_pt always, that disjunct holds for any converged
# covariance when coeff_cov > 1 (production ships 3.0), so a large and steadily
# growing value is EXPECTED and means nn_thresh is not binding. This is the
# measurement that tells you whether the adaptive gate below is worth enabling;
# it is counted whether or not that gate is on.
uint64 cov_escape_admits
# lidar_gate_rejected: cumulative rows dropped by the adaptive (Mahalanobis)
# gate, i.e. |zp| beyond lidar_gate_sigma standard deviations of the predicted
# innovation. Always 0 with lidar_gate_sigma = 0 (the default).
uint64 lidar_gate_rejected
# lidar_gate_disarms: updates where the gate stood down because more than
# lidar_gate_max_reject_frac of the candidate rows would have been rejected --
# read as evidence against the PRIOR rather than the data. A steady stream of
# these means the filter is over-confident (cross-check `nis`), not that the
# scan is bad.
uint64 lidar_gate_disarms
# cov_sanitized: publish batches whose pose/twist covariance failed the
# on-the-wire sanity check (non-finite entry, or non-positive diagonal). MUST
# stay 0; any other value is a bug report, not a tuning signal.
uint64 cov_sanitized
