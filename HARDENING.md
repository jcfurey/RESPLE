# RESPLE Hardening Plan

Phased plan for fixing threading, memory, and accuracy issues in the RESPLE
package. Each phase is independently mergeable and scoped so that later phases
can be deferred or skipped if Phase 0 data says they are not needed.

Companion docs:
- `CLAUDE.md` — architecture, lock ordering, build flags.
- Upstream audit: see the assistant session that produced this plan; the audit
  identified 10 concrete issues ranked by severity, which this plan sequences
  into phases.

## Status at a glance

| Phase | Title | Status | Commit |
| --- | --- | --- | --- |
| 0 | Instrument before changing code | **Complete (code); user bag-replay pending** | `74f9078` |
| 1 | Safety fixes | **Complete** | `512da1d` |
| 2 | Concurrency hardening | Pending Phase 0 data | — |
| 3 | Spline / mapping accuracy | Pending Phase 0 data | — |
| 4 | Diagnostics publisher | Can start after Phase 3 begins | — |
| 5 | Regression tests | Last | — |

Commits are in the `resple` submodule on `develop`, relative to `fced6a1`.

## Guiding principle

**Measure before changing.** The audit identified a dozen plausible hazards.
Before we touch code to fix any of them, we need evidence — from TSan / ASan
reports and from production-bag diagnostics — that each hazard fires in
practice. That is what Phase 0 is for. If you skip it, you will spend time
fixing theoretical races while missing the real ones.

---

## Phase 0 — Instrument before changing code

**Status: code complete, bag replay pending.**
**Commit: `74f9078`.**

### What landed
- **Sanitizer build options.** `ENABLE_TSAN` and `ENABLE_ASAN` in
  `resple/CMakeLists.txt`, mutually exclusive. Both force
  `-O1 -g -fno-omit-frame-pointer` and drop `-ffast-math`; sanitizer flags are
  marked PUBLIC so linking executables pick up the runtime.
- **IEKF failure counter.** `Estimator<>::num_numerical_failures_` is a
  `std::atomic<uint64_t>` incremented at every "IEKF ... update failed
  (numerical)" site (4 sites: LO/LIO × kd-tree/CUDA paths).
- **Diagnostic metrics.** `updateDiagnostics` now publishes:
  - `Spline Knots` — current knot count.
  - `IMU Staging Buffer Size` — depth of `imu_int_buff` (before drain).
  - `IEKF Numerical Failures (LO/LIO, cumulative)` — monotonic.
  - `IEKF Numerical Failures (LO/LIO, last window)` — per-second delta.
  - `Spline Out-of-Range Queries (cumulative)` — (Phase 1) from
    `Association::out_of_range_queries_`.
  - Summary escalates to at least `WARN` when any numerical failure occurs in
    the current window, without overwriting `ERROR`.
- **CLAUDE.md.** Authoritative package doc: threading model, four-mutex lock
  ordering, build flag rationale, known-hazard list with status.

### What the operator must do next
Phase 0 is instrumentation, not a fix. The signal it produces is what drives
Phase 2/3 decisions.

1. **Run a representative bag under TSan.** From inside the dev container:
   ```bash
   colcon build --packages-select resple \
     --cmake-args -DENABLE_TSAN=ON -DENABLE_NATIVE_ARCH=OFF
   source install/setup.bash
   TSAN_OPTIONS="halt_on_error=0:second_deadlock_stack=1" \
     ros2 launch bringup_robot_erdc play_bag.launch.xml
   ```
   Save the stderr to `~/phase0_tsan.log`. Any real data race prints a
   stack trace.
2. **Run a representative bag under ASan + UBSan.** Same pattern with
   `-DENABLE_ASAN=ON`. Catches UAF after ikd-Tree flatten, vector
   iterator invalidation, UB from signed overflow / strict aliasing.
3. **Observe production diagnostics for 10+ minutes per environment
   (tunnel, outdoor, sim).** Record:
   - `Spline Knots` growth rate. Decides whether Phase 3 knot pruning is
     urgent (if growth is >>> bag duration × 100 Hz, something is wrong).
   - `IMU Staging Buffer Size` peak under load. Decides whether Phase 2
     bounded-buffer work is urgent.
   - `IEKF Numerical Failures (cumulative)` over a long run. If non-zero,
     Phase 3 divergence detection moves up.
   - `Spline Out-of-Range Queries` — if non-zero, the deskew window
     bound is being violated and upstream timestamps need inspection.

### Decision gates coming out of Phase 0
- **If TSan flags `Nearest_Search` vs. rebuild thread:** Phase 2 item #1
  becomes urgent (might be production-blocking).
- **If ASan flags UAF:** likely a Phase 2 concurrency issue; prioritize above
  Phase 3 accuracy work.
- **If knot growth is unbounded:** Phase 3 item #1 (knot pruning) moves up.
- **If IEKF numerical failures occur regularly:** Phase 3 item #3 (divergence
  detection) moves up.
- **If no sanitizer reports and no growth concerns:** focus Phase 3 on
  accuracy (plane fit, outlier rejection) and skip bounded-buffer work.

---

## Phase 1 — Safety fixes

**Status: complete.**
**Commit: `512da1d`.**

### What landed
- **`SplineState.h` asserts replaced.** All five `assert()` sites converted to
  `RESPLE_LOG_INVARIANT_ONCE` + safe fallback. The default RelWithDebInfo
  build sets `-DNDEBUG`, so the old asserts were no-ops and the code after
  them silently performed UB (out-of-bounds `q_knots` writes, uninitialized
  `cp0` in quaternion interpolation). Each site now logs once per process and
  either early-returns or uses an identity-quaternion fallback.
- **`Association::pointBodyToWorld` bounds check.** Explicit range check
  against `[spline->minTimeNs(), spline->maxTimeNs()]`, with a cumulative
  counter on the inline-static `Association::out_of_range_queries_`. The
  active-window guard in `findCorresp` remains, but the deskew pass in
  `RESPLE::processData` transforms every `pt_meas` entry unconditionally —
  this counter surfaces out-of-window queries in diagnostics rather than
  silently extrapolating.
- **Eigen zero-init.** `Estimator<>::cov_rcp`, `cov_sys`, `a_mat`, `KH`,
  `cov_post_` explicitly default-initialized to zero via in-class
  initializers. The old code relied on `EIGEN_INITIALIZE_MATRICES_BY_NAN`
  which is Debug-only; Release builds had uninitialized matrices until
  `setState()` / `update()` first wrote them.
- **Future access gated on atomic.** `std::atomic<bool> map_update_pending_`
  replaces the `std::future::valid()` pattern in `on_deactivate`,
  `on_cleanup`, and the processData loop. Set `true` before `std::async`,
  cleared as the lambda's final step. Sidesteps the standard-ambiguous
  future read.
- **`-ffast-math` dropped.** Removed from both `ENABLE_NATIVE_ARCH` and the
  plain `-O3` path. The Joseph-form `(I-KH)P(I-KH)^T+KRK^T` update's
  PSD-preservation argument depends on IEEE-754 associativity; `-ffast-math`
  also silences NaN/Inf divergence checks. `-Wextra -Wshadow` added to
  match workspace convention.

### Known follow-ups
- `Mapping.cpp:242` triggers `-Wreorder` (pre-existing): `node_handle_`
  declared after `lidar_qos` but initialized before it. Benign here but worth
  fixing for -Wall hygiene. Not on the hardening critical path.
- `RESPLE.cpp:405` has an unused `use_gpu` in the non-CUDA branch. Pre-existing.
  Silence with `[[maybe_unused]]` if noise is annoying.

---

## Phase 2 — Concurrency hardening

**Status: pending Phase 0 data.**

Three independent items. Do #1 if TSan flags a race; do #2 regardless; do #3
if Phase 0 diagnostics show buffer growth under load.

### 2.1 Verify ikd-Tree `Nearest_Search` lock contract

The IEKF takes `mtx_map_` as a shared lock and then runs
`#pragma omp parallel for` over `findCorresp`, each worker calling
`Nearest_Search` on the shared tree. This is only safe if the ikd-Tree
internally serializes its own rebuild-thread writes against concurrent
searches. The audit flagged this as `HIGH` severity but unverified.

**Sub-tasks:**
1. Read `resple/include/ikd-Tree/ikd_Tree.cpp` — trace the lock discipline
   for `Nearest_Search` vs. `Add_Points` vs. the background rebuild thread.
2. Write a gtest that issues N parallel `Nearest_Search` calls while
   another thread issues `Add_Points`. Run under TSan.
3. If unsafe, pick a mitigation:
   - **(a) Serialize IEKF parallel-for with a unique lock.** Simple; cost is
     losing the parallelism for k-NN.
   - **(b) Flatten the active subtree into a read-only buffer** before the
     IEKF parallel section; search that buffer instead. More code, but keeps
     the parallelism.

**Decision gate:** only do (a)/(b) if the stress test or TSan confirms a
real race. Do not preempt without evidence.

### 2.2 Initialization state machine

`if_init_filter` + `if_init_map` + `localmap_initialized_` is a three-bool
state machine whose ordering rules are correct today but easily broken by
future edits. Convert to a single `std::atomic<State>`:

```cpp
enum class State : uint8_t { UNCONFIGURED, INITIALIZING, READY };
std::atomic<State> state_{State::UNCONFIGURED};
```

Callbacks early-return unless `state_.load() == State::READY`. IMU callback's
current snapshot-under-mutex pattern becomes a compare-exchange against the
state atomic — more explicit, less fragile.

**Cost:** small refactor across `on_activate`, `on_deactivate`, `on_cleanup`,
`getImuCallback`, `initialization()`. Can be done without Phase 0 data.

### 2.3 Bounded input buffers

`pc_buff` (per-LiDAR) and `imu_int_buff` are unbounded `std::deque`s. If the
worker stalls, memory grows without backpressure. Fix:
- Parameter-configurable max size (default: sized from Phase 0 measured peak
  × 2).
- Drop-oldest on overflow.
- Counter published in diagnostics (`… dropped this window`).

**Decision gate:** only do this if Phase 0 shows the buffers trend up under
normal operation. On a clean system they should empty every IEKF cycle.

---

## Phase 3 — Spline / mapping accuracy

**Status: pending Phase 0 data.**

Four items. Each is a distinct PR with a before/after trajectory-plot
regression before merging.

### 3.1 Sliding-window knot pruning

`SplineState::t_knots` / `q_knots` / `ort_delta` grow indefinitely. Define a
retention window (something like `max(5 × spline_order, active_measurement_span)`
knots) and prune from the front. Must guard any consumer that reads historical
knots — notably `getSplineMsg` and any plugin that walks backward from the
current state.

**Complexity: medium.** Risks invariant violations in `updateKnots`,
`setOneStateKnot`, and the IEKF's `getRCPs()`. Write unit tests before the
refactor.

**Decision gate:** only schedule if Phase 0 shows knot count growing
unboundedly relative to wall-clock runtime. On a clean system, the growth is
`knot_hz × seconds` = `100 × t` and pruning keeps the working set small.

### 3.2 Plane-fit hardening

`Association::findCorresp` uses only a point-to-plane distance threshold
(`pd2 < 5`, scaled by range²) and a hardcoded `esti_plane` threshold `0.1f`.
No eigenvalue-ratio test (no degeneracy rejection for edge points or noise
clusters). No counters for dropped candidates.

**Work:**
- Add eigenvalue ratio check `λ0 / λ2 > 20` (or parameter) in `esti_plane` or
  after.
- Expose thresholds as parameters in
  `src/settings/params/localization/resple.yaml`.
- Publish per-scan counters: `{candidates, passed_distance, passed_plane,
  used_in_IEKF}` — routed through the Phase 4 diagnostic topic.

**Complexity: low-medium.** Numerical impact depends on parameter choices;
benchmark against a known-good bag.

### 3.3 Divergence detection and recovery

After each IEKF pass, the filter's health is invisible externally. Bad
posterior covariance (collapse or NaN) propagates into published odometry
and downstream (Sierra) without any warning.

**Work:**
- Compute posterior covariance trace and `λ_min` (via `SelfAdjointEigenSolver`).
- Check: trace finite, `λ_min > 0`, no NaN in state.
- Publish a `/localization/resple/status` topic with the health bool + the
  metrics.
- Action on trip: either (a) hold last estimate + skip publish until a stable
  scan recovers it, or (b) reset filter with the last good pose.

**Complexity: medium.** Eigendecomposition on 24x24 / 30x30 is cheap
(< 1 ms). Choice of (a) vs. (b) is a policy decision — (a) is safer
operationally, (b) recovers faster from a real divergence.

### 3.4 Map pruning policy

`lasermapFovSegment` uses a hardcoded cube around the current pose (size set
by `cube_len` param). Two issues: no radius-based pruning, and no temporal
decay (dynamic obstacles never drop out).

**Work:**
- Add radius-based pruning (drop cells > `prune_radius` m from current pose)
  as an alternative or supplement to the cube.
- Optional temporal decay: drop cells not hit in `temporal_half_life` seconds
  (requires a per-cell timestamp — potentially expensive, defer).

**Complexity: medium-low.** Radius is cheap; temporal decay needs storage.

---

## Phase 4 — Diagnostics publisher

**Status: can start after Phase 3 begins.**

Today, diagnostics scatter across `diagnostic_updater` fields and ROS logger
output. Consolidate into a single topic for easier monitoring and for Sierra
to consume as a trust signal.

### Proposal
New topic `/localization/resple/diagnostics` (custom msg or
`diagnostic_msgs/DiagnosticArray`) with:

| Field | Source |
| --- | --- |
| `knot_count` | `spline->numKnots()` |
| `imu_buffer_depth` | `imu_int_buff.size()` |
| `lidar_buffer_depth` | sum of `pc_buff.size()` per LiDAR |
| `iekf_iterations` | `estimator_*.n_iter` per frame |
| `iekf_numerical_failure` | bool, this cycle |
| `cov_trace`, `cov_lambda_min` | from Phase 3.3 |
| `plane_fit_counters` | from Phase 3.2 |
| `deskew_oob_count` | `Association::out_of_range_queries_` |
| `map_size` | `ikdtree` node count |
| `stage_timing_ms` | per-stage (drain, IEKF, deskew, map update) |

### Consumers
- Operator dashboards (Foxglove, PlotJuggler).
- Sierra: may use covariance trace to adjust `resple_odom` plugin's trust
  weight dynamically.

---

## Phase 5 — Regression tests

**Status: last.**

The package ships no tests today. Each earlier phase should contribute tests;
Phase 5 is about filling gaps and wiring them into CI.

### Targets

| Test | Scope |
| --- | --- |
| `SplineState` unit | Bounds, bootstrap, (future) pruning |
| `Association::findCorresp` stress | Multi-threaded `Nearest_Search` under concurrent mutation (validates Phase 2.1) |
| `Estimator<>` unit | Joseph-form PSD property preserved across iterations |
| Bag-replay CI smoke | Replay short sim bag, compare final pose to tolerance, fail on divergence flag |
| Sanitizer CI | Nightly build + smoke test with `ENABLE_TSAN=ON` and `ENABLE_ASAN=ON` |

### Wire-up
Flip `BUILD_TESTING` to `ON` for this package in `colcon_defaults.yaml`
(currently OFF workspace-wide). Add the gtest binaries to `ament_add_gtest`.

---

## Open questions for future work

Out of scope of this plan but worth noting:

1. **Should RESPLE own the map, or should mapping live in a separate node?**
   Today the kd-tree is a global `ikdtree` in `RESPLE.cpp`. A separation
   would let mapping degrade independently of the filter. Non-trivial
   architectural change.
2. **GPU k-NN path activation.** `ENABLE_CUDA` exists but is off in this
   workspace. If Phase 2/3 benchmarking shows CPU k-NN is the bottleneck,
   this becomes interesting.
3. **Multi-LiDAR support.** The upstream package supports MLO / MLIO. We only
   use a single Ouster. If we add a second LiDAR (e.g. back-facing), the
   lock-ordering rules in CLAUDE.md need a re-audit.
