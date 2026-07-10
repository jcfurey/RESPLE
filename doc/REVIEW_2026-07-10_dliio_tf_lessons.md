# RESPLE review — dliio TF / output-shape lessons (2026-07-10)

Status: **both features landed (default-off) + audit clean**; the X-ICP ternary
gate port remains deferred to its own n≥24 campaign.

Companion to `REVIEW_2026-06-27_dliio_lessons_and_bugs.md` (degeneracy-stack
lessons, bugs A1–A4). This pass asked the complementary question — what does
dliio do *structurally* (TF, extrinsics, output shape) that RESPLE should
adopt — plus a one-time audit for the bug classes found while upstreaming
dliio's intensity/reflectivity work (gglaspell/dliio PRs #1/#2).

On tf2 specifically the lesson flow mostly runs the OTHER way: dliio
broadcasts unconditionally (no `publish_tf` toggle, no invert, no ownership
detection — it would have silently fought the 07052026 bag's tree, the exact
failure `utils/tf_ownership.h` catches), and it re-broadcasts its *static*
sensor extrinsics on dynamic `/tf` at IMU rate. Neither habit was ported.
Two structural ideas were worth taking, both implemented better than the
original.

---

## Landed 1 — `publish_extrinsic_tf` (default `false`)

**dliio's good idea:** its param extrinsics are broadcast as
`base_link → imu` / `base_link → lidar`, so its TF tree is complete out of
the box — any consumer can transform raw sensor data with no external helper.

**RESPLE's gap:** in the YAML `q_lb`/`t_lb` convention (`tf_extrinsics:
false`, or the `tf_wait_timeout` fallback) nobody publishes the sensor
frames. `scripts/run_replay.sh` / `run_accuracy.sh` (test_ws) each spawn a
raw `tf2_ros/static_transform_publisher` — the helper behind the leaked
stale-extrinsic race that forced the `pkill -f` cleanup dance.

**Implementation** (`RESPLE.cpp latchExtrinsicTf`, called from
`updateLidarTransform`'s two YAML-convention paths): latch
`frame_id → <cloud header frame>` once on `/tf_static` with `(q_bl, t_bl)` —
exactly the `T_base←sensor` `pointBodyToWorld` applies, so TF consumers and
the estimator agree by construction. Deliberate deltas vs dliio:

- **`/tf_static` latch, not per-cycle `/tf` re-broadcast** — the extrinsic is
  static; `/tf` traffic is untouched.
- **LiDAR frames only.** RESPLE has no YAML IMU extrinsic; asserting an
  identity `base → imu` we don't know would mislead downstream (dliio's
  06042026 180°-yaw-fused-as-identity bug is the cautionary tale).
- **Collision guard:** skipped with a WARN if the sensor frame already exists
  in the TF tree (a bag's own `/tf_static`, a URDF). Best-effort — prefix
  RESPLE's frames when a bag carries its own tree (`config_07052026.yaml`
  pattern).
- **Default off** — house rule for new behaviour; enable per dataset config.

## Landed 2 — `odom/dense_pub_hz` (default `0` = off)

**dliio's good idea:** odom + TF at ~100 Hz (IMU rate), stamped with data
time, so a downstream `lookupTransform` at a camera/IMU stamp interpolates a
dense chain instead of extrapolating.

**Correction found while smoke-testing:** RESPLE had this shape all along.
`publishPoseAndTf` runs once per `collectMeasurements` iteration — one knot
per call — so steady-state output is already knot-rate (`knot_hz: 100` in
every shipped config; measured **101.7 Hz on the R_Campus baseline leg**,
not the ~scan-rate the initial assessment assumed). The dliio parity gap is
narrower: whenever the edge advances by MORE than one knot per publish
(`propRCP` jumping a scan gap, overload shedding — hazard 69 — or an
NIS-hold release), the duplicate gate emits a single pose at the new edge
and leaves a hole in the chain exactly where lookups are most fragile.

**Implementation** (`publishPoseAndTf` → loop over `publishPoseAtNs`;
pure sample-time core in `utils/dense_pub.h`, unit-tested in
`test/test_dense_pub.cpp`): a publish-density **floor** — back-fill the
newly-valid segment `(last publish, edge)` at the configured rate by spline
interpolation, data-time stamps. No-op in steady state at `dense_pub_hz ≤
knot_hz`; values above `knot_hz` upsample sub-knot. Deltas vs dliio:

- **Interpolated, never extrapolated** — dliio publishes IMU-propagated
  (predicted) state; the spline samples are estimated history. Latency is
  unchanged. (Matching dliio's *freshness* would mean an IMU propagation on
  top of the spline — estimator work, not done.)
- **Bounded burst:** back-fill after a stall / NIS-hold release / bag restart
  is capped at 1 s; the first-ever publish back-fills nothing. The NIS `hold`
  gate sits upstream of the whole publish call, so a held window is never
  densely back-filled.
- Stamps stay strictly monotonic (tf2 rejects duplicates); every dense TF
  stamp goes through `tf_own_monitor_.notePublished` so the ownership guard's
  self-stamp ring keeps working (256 slots ≫ 100 Hz × loopback latency;
  rate clamped ≤ 1 kHz).
- Covariance: all samples in a batch carry the batch posterior — there is one
  IEKF posterior per batch and these stamps belong to it.

**Smoke (R_Campus, LIO, container Release build):** dense leg — `/odom`
100.1 Hz, `/tf` 101.6 Hz, `/tf_static` latched `base_link → livox_frame`
with exactly `t_bl = −t_lb = [0.04165, 0.02326, −0.0284]`, 511 stamps 0
non-monotonic, node alive. Baseline leg — no latch, no dense log line,
101.7 Hz knot-rate output unchanged, 510 stamps 0 non-monotonic. The
throttled `IMU health faults=0x10` WARN appears in both legs (pre-existing,
wall-clock replay artifact).

## Audit — dliio PR-work bug classes, RESPLE-side (all clean)

| dliio bug (fixed upstream in PRs #1/#2) | RESPLE exposure |
| --- | --- |
| `pcl::PointCloud(1, N)` ctor arg order → wrong organized flag | No `PointCloud(w, h)` ctor use anywhere in `resple/src` + includes |
| VoxelGrid silently zeroing custom point fields | Downsampling runs on standard `pcl::PointXYZINormal` (all fields registered upstream in PCL); the custom-struct trap doesn't apply. Mapping already accounts for VoxelGrid reorder/average in scan-end times (2026-07-02 fix) |
| Sensor-type detection dereferencing `points[0]` on an empty cloud | Already hazard #28 (fixed post-1.5) — independently the same bug |
| Configured channel/field never validated against actual PointCloud2 fields | Generic ingestion resolves x/y/z + time/intensity fields by name at runtime (`utils/point_cloud_adapter.h`, unit-tested) |
| Stale state read when an update is skipped (`converged_` class) | The NIS analogue was found and fixed as bug A1 in the 2026-06-27 review |

## Deferred — X-ICP ternary localizability gate port

Still the highest-value estimator lesson: dliio's ternary gate is validated
at n=24 (Fisher p=0.0055) on the 06042026 tunnel while RESPLE's scalar
levers went ~180× worse in the 2026-07-02 A/B and `plane_min_cond_ratio`
0.05 froze it at the origin. Design notes live in
`REVIEW_2026-06-27_dliio_lessons_and_bugs.md`; any port must ship default-off
and be judged on the test_ws n≥24 harness (`scripts/resple_tunnel_ab.sh`),
not a smoke run.
