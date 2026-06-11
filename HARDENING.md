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
| 0   | Instrument before changing code | **Complete (code)**; bag replay folded into §6.1 | `74f9078` |
| 1   | Safety fixes (initial) | **Complete** | `512da1d` |
| 1.5 | Defensive crash-hardening (3-pass) | **Complete** | `14e9be8` |
| 1.6 | Bug-chase session 2026-05-01: `__libc_free` SIGSEGV in `KD_TREE::Add_Points` | **Complete** — diagnosis later independently confirmed by the Phase 5 ASan gate; see the updated "What's still open" | — |
| 2   | Concurrency hardening | **Complete** — 2.1/2.2 via 1.5; 2.4 + 2.5 #1 + 2.5 #2 + 2.6 + 2.7 fixed + TSan-verified; 2.3 capability landed (default-off per gate) | — |
| 3   | Spline / mapping accuracy | **Complete** — 3.1 pruning; 3.2 parameterized + instrumented; 3.3 detection + recovery modes; 3.4 radius pruning (opt-in). Tuning passes → §6.3/§6.4 | — |
| 4   | Diagnostics publisher | **Complete** — `estimate_msgs/Diagnostics` on `resple_diagnostics`, ~20 Hz typed | — |
| 5   | Regression tests | **Complete except §6.2** — ROS-free + ASan/UBSan + ikd-Tree TSan CI gates | — |
| 6   | Bag-gated validation & tuning | **In progress (bag available 2026-06-10)** — §6.1 TSan leg DONE: 4 real concurrency bugs found+fixed on the HelmDyn01 LIO replay (see §6.1 results); §6.1 ASan leg, §6.2 CI smoke, §6.3 plane-fit tuning, §6.4 recovery policy still open | — |

Commits are in the `resple` submodule on `develop`, relative to `fced6a1`.

Phase 1.5 was added after Phase 0 instrumentation indicated production crashes
were still occurring (deterministic mid-run SIGSEGV reported). Without a
backtrace, three review passes applied defensive fixes for every plausible
SIGSEGV/SIGABRT/race candidate identified by code reading. See "Phase 1.5"
section below for the full list — 13 fixes across 5 files.

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
- `RESPLE.cpp:405` had an unused `use_gpu` in the non-CUDA branch. Fixed in
  Phase 1.5 commit (`[[maybe_unused]]`).

---

## Phase 1.5 — Defensive crash-hardening (3-pass)

**Status: complete.**
**Commit: `14e9be8`.**

Triggered by an operator report of a deterministic mid-run SIGSEGV after
Phase 0+1 landed. No backtrace available. Three review passes covered every
plausible candidate identified by code reading. All fixes are defensive —
log + continue with degraded value rather than UB / silent crash. None
change steady-state filter behavior.

### Pass 1 — RESPLE node, primary candidates

Files: `Association.h`, `SplineState.h`, `RESPLE.cpp`.

| Fix | Hazard | Resolution |
| --- | --- | --- |
| **A** | `Association::pointBodyToWorld` only logged out-of-range queries (Phase 1 fix), then still called `itpPose` with the bad t_ns → OOB-read on `t_knots[idx0+i]` → SIGSEGV. | Clamp `t_ns` to `[minTimeNs, maxTimeNs]` before `itpPose`. Counter increments on clamp; log throttled. |
| **B** | `SplineState::itpPose` and `prepareInterpolation` indexed `t_knots[idx0+i]` without bounds-check. For `t_ns > maxTimeNs()`, `idx0 + (n_active-1)` ≥ `num_knot` → deque OOB. | Belt-and-braces clamp of `t_ns` at function entry; secondary `n_active_safe` cap against `knots.size()`. Logs once-per-process via `RESPLE_LOG_INVARIANT_ONCE` if hit. |
| **C** | Worker's `processData` read `imu_int_buff.empty()` and bare-bool `if_init_filter` / `if_lidar_only` / `if_init_map` / `localmap_initialized_` without `m_buff` lock. Real data races on deque internals + bool reads. | Snapshot `imu_int_buff` size under `m_buff` before drain decision. Convert four bools to `std::atomic<bool>` (operator T() / operator= preserves existing call sites). |
| **D** | All 6 lidar callbacks + IMU callback called `lidars.at(name)` without try/catch. An unexpected frame_id → `std::out_of_range` → executor → `std::terminate` → SIGABRT. | Wrap each callback body in `try { ... } catch (const std::exception& e) { RCLCPP_WARN_THROTTLE(...); }`. |
| **E** | `Subscription::reset()` in `on_deactivate` does NOT synchronize with executor-dispatched callbacks. `on_cleanup`'s `lidars_data.clear()` could free a `LidarData` while a callback held a ref to its `mtx_pc` → SIGSEGV. | 100 ms drain barrier in `on_deactivate` after subscription resets. Documented as temporary until rclcpp exposes a callback-group `wait_for` API in Jazzy. |
| **F** | `processing_thread_ = std::thread(...)` in `on_activate` would call `std::terminate` if the thread was still joinable from a failed prior transition. | Refuse re-activation while `processing_thread_.joinable()` — return `FAILURE` cleanly. |

### Pass 2 — Mapping node, lifecycle re-entry, ikd-Tree

Files: `RESPLE.cpp`, `Mapping.cpp`, `ikd_Tree.cpp`.

| Fix | Hazard | Resolution |
| --- | --- | --- |
| **R1+R2** | `RESPLE.cpp:91/99` (`on_configure`) and `:185` (`on_activate`) called `declare_parameter` directly. A second invocation (after `on_cleanup`) throws `ParameterAlreadyDeclaredException` → SIGABRT. The other params in `readParameters()` use `CommonUtils::readParam` which has a `has_parameter` guard. | Add `has_parameter` guards on the three direct calls; fall back to `get_parameter().as_int() / .as_string_array()`. |
| **R3** | `save_map_action_server_`, `tf_buffer_`, `tf_listener_` created in `on_configure` but not reset in `on_cleanup`. A re-`configure` cycle would orphan the original action server (potential goal-namespace conflict) and pin the TF listener thread. | Reset all three in `on_cleanup`. |
| **M1+M2** | `Mapping.cpp` `if_init_succeed` was bare bool, written by `startCallBack` (executor) and read by `process()` (worker). Worse: `startCallBack` called `spline_active_.init(...)` while `process()` could be reading `spline_active_.numKnots()` etc. — race on `SplineState` internals. | Convert `if_init_succeed` to `std::atomic<bool>`. `startCallBack` now does `init()` under `m_spline` and release-stores the flag; `process()` acquire-loads it. The synchronizes-with relationship guarantees init writes are visible before the worker reads `spline_active_`. |
| **M3** | Same callback-drain hazard as RESPLE Fix E, in Mapping's `on_deactivate`. | 100 ms sleep + clear `if_init_succeed` after subscription resets. |
| **K1** | `ikd-Tree::Nearest_Search` (line 404) checked `Rebuild_Ptr == nullptr \|\| *Rebuild_Ptr != Root_Node` lock-free, then conditionally took `search_rw_mutex_` shared. The rebuild thread can clear `Rebuild_Ptr` between the check and the search → search runs without the lock while rebuild mutates the tree → UAF → SIGSEGV. HIGH severity per Phase 2.1 audit, never verified — fixed defensively. | Remove the racy fast-path; always take `search_rw_mutex_` shared. Tiny perf cost (uncontended in steady state; rebuild only takes unique briefly during the subtree swap). |

### Pass 3 — Latent issues found in fresh walk

Files: `RESPLE.cpp`, `common_utils.h`, `Mapping.cpp`.

| Fix | Hazard | Resolution |
| --- | --- | --- |
| **getPositionLiDAR side-effect** | Called from the async map-update lambda (under `mtx_map_` unique only), it called `propRCP(t_ns)` which mutates `Estimator::cov_rcp` (`cov_rcp += cov_sys`). Race-free only because the IEKF needs `mtx_map_` shared and was blocked — the implicit lock-coupling was fragile and undocumented. | Removed `propRCP` call from `getPositionLiDAR`; only caller (`lasermapFovSegment`) passes `spline->maxTimeNs()` so the spline never grew anyway. Function is now pure-read, lock discipline matches the documented contract. |
| **Alignment static_asserts** | `Eigen::aligned_deque<PointData>` accessed via OpenMP — analytically safe (`sizeof(T)` is a multiple of `alignof(T)` per the standard), but the invariant was not enforced. Future addition of a non-aligned member could silently misalign deque chunk elements → SIMD UB. | `static_assert(sizeof(T) % alignof(T) == 0)` for `PointData` and `ImuData`. Future-proof against regressions. |
| **M4 — RAII for `lock_mappings`** | `Mapping.cpp` had `lock_mappings()` / `unlock_mappings()` non-RAII pairs. A throw inside `publishPath` / `displayControlPoints` / `pubOdom` between them would leak per-map mutexes → next iteration deadlocks on its own `mtx_pc.lock()`. | Replace with a `ScopedMappingsLock` RAII helper that locks all maps on construction and releases (in reverse order) on destruction. |

### Files touched

| File | Passes |
| --- | --- |
| `resple/include/Association.h` | 1 |
| `resple/include/SplineState.h` | 1 |
| `resple/include/utils/common_utils.h` | 3 |
| `resple/include/ikd-Tree/ikd_Tree.cpp` | 2 |
| `resple/src/RESPLE.cpp` | 1, 2, 3 |
| `resple/src/Mapping.cpp` | 2, 3 |

### What this catches that Phase 0/1 didn't

- Phase 1 made `pointBodyToWorld` log out-of-range queries but did not actually
  prevent the OOB; Pass 1 Fix A clamps it.
- Phase 1 did not address bare-bool data races on the init flags; Pass 1 Fix C
  did (atomic conversion, no behavioral change).
- Phase 1 did not address the lifecycle re-entry SIGABRT path on
  `declare_parameter`; Pass 2 R1/R2 do.
- Phase 2.1 was deferred pending TSan evidence; Pass 2 K1 fixed it
  defensively.
- Mapping.cpp was entirely out of scope of Phases 0–1; Pass 2 (M1–M3) and
  Pass 3 (M4) covered the equivalent classes of bugs there.

### What's intentionally still TODO

- **Phase 0 sanitizer bag-replay** — operator task, see Phase 0. Highest
  remaining ROI: would surface anything not yet caught by code reading.
- **`respleCrashHandler` backtrace from a real crash** — the crash handler
  is installed (Phase 0); if a SIGSEGV survives Phase 1.5, the stderr
  backtrace tells us exactly where, and the next pass can be data-driven
  rather than speculative.
- **Phase 2.2 full state-machine refactor** (`enum class State { ... }` with
  compare-exchange transitions). Pass 1 Fix C made the bools atomic, which
  closes the immediate race. The full refactor is still a code-clarity win
  but no longer urgent.
- **Phase 2.3 bounded input buffers** — Pass 0 diagnostics need to fire
  first to confirm buffers actually grow under load.
- **Phase 3.x and Phase 4** — unchanged; these need data + design discussion.

---

## Phase 1.6 — Bug-chase session, 2026-05-01

**Status: complete in this commit. Did NOT eliminate the production
SIGSEGV; root cause localized via `addr2line`, fix targeted at last edit.**

### Trigger

User attempted to switch the active production stack from DLIO to RESPLE
(env.d/20-features.env: `run_resple=True`, env.d/30-algorithms.env:
`run_dlio=False`, odom_localization.yaml: odom1 → /localization/resple/odometry).
RESPLE crashed reproducibly with exit code -11 (SIGSEGV) on the first
`mapIncremental` call after gravity alignment. Crash signature stable
across all configurations (sim/hw not relevant — sim was used throughout).

### Diagnostic timeline (what we ruled out, in order)

1. **`respleCrashHandler` was firing** — backtrace was being captured but
   getting buried by the `respawn=True` log-loop. Once visible, every
   crash had the same stack: `__libc_free` ← inside `KD_TREE::Add_Points`
   at offset `+0x267`, called from `mapIncremental + 0x389` (later +0x3d0
   after the NaN-skip code expanded the function).
2. **Lifecycle return-value path** — patched `RESPLE.cpp` and `Mapping.cpp`
   `main()` to check `configure()` / `activate()` returns and exit non-zero
   on failure. Confirmed lifecycle reached `active` cleanly; not the issue.
3. **AVX / `-march=native` PCL alignment mismatch** — disabled
   `-DENABLE_NATIVE_ARCH=OFF`. Same crash. Not it.
4. **OpenMP-concurrent kd-tree access** — set `num_threads=1` in
   resple.yaml. Worker crash unchanged; rebuild thread crash gone (so OMP
   races are real but not THIS bug).
5. **Inline `Rebuild` path in `Add_by_point`** — diagnostic patch to force
   `Rebuild` always onto multi-thread queue. Worker crash unchanged
   (inline rebuild bypassed; another path hit the same free).
6. **Try `-O0` build** — added `ENABLE_DEBUG_O0`. NO crash, but worker too
   slow to keep up with 10 Hz scans → `map_update_future_.wait()` deadlock.
   Confirmed UB-exposed-by-O3. Added `wait_for(5s)` timeout (Fix #10) and
   try/catch around worker iteration body (Fix #11) so future stalls
   degrade visibly instead of silent freeze.
7. **`-O1` build** — added `ENABLE_DEBUG_O1` with
   `-fno-strict-aliasing -fno-tree-vectorize`. Build was a no-op (CMake
   cache held the previous flag); user docker broke before retesting.
8. **`-O3 -g3` build** — added `ENABLE_DEBUG_O3G`. Crash reproduced with
   line numbers preserved. **`addr2line` resolved the inlined chain**:

```
KD_TREE<PointXYZINormal>::Add_Points          ikd_Tree.cpp:485
↑ std::vector<...>::~vector()                 stl_vector.h:738
↑ std::_Vector_base<...>::_M_deallocate       stl_vector.h:390
↑ Eigen::aligned_allocator::deallocate        Memory.h:921
↑ Eigen::internal::aligned_free               Memory.h:206
↑ Eigen::internal::handmade_aligned_free      Memory.h:118  ← faults here
↑ __libc_free
```

Line 485 is `PointVector().swap(Downsample_Storage)` — temp's destructor
tried to free Downsample_Storage's previously-held storage.

### Root cause (working theory, supported by the symbolicated trace)

`Eigen::internal::handmade_aligned_free` recovers the original malloc'd
pointer by reading `*(reinterpret_cast<void**>(ptr) - 1)` — i.e., it
assumes the user pointer was returned by `handmade_aligned_malloc`,
which stored a header word at offset `-sizeof(void*)`.

If the matching ALLOCATION instead went through any other Eigen path
(`posix_memalign` or `_mm_malloc`), the bytes immediately before `ptr`
are not a stored header — they're either the previous chunk's trailing
data or padding from the system allocator. Reading that as a pointer
yields garbage, and `std::free(garbage)` faults inside `__libc_free`.

This dispatch mismatch can occur when Eigen's preprocessor gating for
`EIGEN_HAS_POSIX_MEMALIGN` / `EIGEN_HAS_MM_MALLOC` evaluates differently
across translation units. With ODR-merged template instantiations (the
allocator's allocate/deallocate are inline templates), the linker can
pick one TU's `allocate` and another TU's `deallocate` — different
paths, incompatible at the chunk-header level.

The trace confirms `aligned_free` is dispatching to `handmade_aligned_free`
in our build, which means at least one TU sees neither
`EIGEN_HAS_POSIX_MEMALIGN` nor `EIGEN_HAS_MM_MALLOC` defined. That
inconsistency is the bug.

### Fix landed

In `resple/CMakeLists.txt` (see the annotated comment block above the
`target_compile_definitions` call — that block is the source of truth):

- **First attempt (superseded, do not reintroduce):**
  `EIGEN_HAS_POSIX_MEMALIGN=1 EIGEN_HAS_MM_MALLOC=0`. This *did not work* —
  Eigen's `Memory.h` `#define`s those macros unconditionally from its own
  `_GNU_SOURCE` / `_POSIX_ADVISORY_INFO` feature test, so a `-D` from the
  command line gets overwritten when the header is preprocessed, and dispatch
  still reached `handmade_aligned_free`.
- **Current fix (in tree):**
  ```cmake
  target_compile_definitions(${PROJECT_NAME} PUBLIC
      _GNU_SOURCE
      EIGEN_MALLOC_ALREADY_ALIGNED=1
      EIGEN_DEFAULT_ALIGN_BYTES=16)
  ```
  `_GNU_SOURCE` satisfies Eigen's own `posix_memalign` feature test;
  `EIGEN_MALLOC_ALREADY_ALIGNED=1` short-circuits `aligned_free` to plain
  `std::free` regardless of the other macros (the actual safety net); and
  `EIGEN_DEFAULT_ALIGN_BYTES=16` pins alignment to match PCL and what
  `std::malloc` delivers on x86_64 glibc, preventing over-alignment to 32
  (AVX) from re-triggering the aligned-malloc dispatch.

**Status at commit time: built and installed; not yet verified to eliminate
the crash in a sim run** (work committed in progress). If it still fires, the
next escalation is `EIGEN_MAX_ALIGN_BYTES=0` (disable Eigen alignment
entirely), but that costs vectorization — try only if the current defines
don't hold.

### Other defensive fixes landed alongside (also in this commit)

These don't address the root SIGSEGV but harden RESPLE against related
classes of failure that surfaced during the bug-chase. All have
standalone justification.

| Fix | File | Hazard addressed |
| --- | --- | --- |
| K1 extension | `ikd_Tree.cpp` (Add_Points / Add_Point_Boxes / Delete_Points / Delete_Point_Boxes / Box_Search / Radius_Search) | Same UAF race as Phase 1.5 K1 in `Nearest_Search`: lock-free fast-path check vs rebuild-thread subtree swap. Per-branch shared lock pattern; slow path still uses `working_flag_mutex` to avoid lock-order inversion |
| Async lambda try/catch | `RESPLE.cpp` async map-update lambda | Throw inside `mapIncremental` / `lasermapFovSegment` / `publishFrameWorld` would silently stick `map_update_pending_=true` and lose the exception in the future destructor. Now logged + always cleared |
| `Mapping::main()` LidarConfig try/catch | `Mapping.cpp` | `LidarConfig` ctor `q_lb_v.at(3)` throws on truncated YAML → uncaught → terminate → SIGABRT with no log. Now `RCLCPP_FATAL` + clean exit code |
| Lifecycle return-value check | `RESPLE.cpp` + `Mapping.cpp` `main()` | `configure()` / `activate()` ignored returns; failed transitions left executor spinning on half-init node. Now check id, log id+label, exit non-zero on failure |
| `numeric_limits::lowest()` | `RESPLE.cpp` `lasermapFovSegment` | `pos_lidar_max` was init'd with `min()` (smallest positive), not `lowest()` (most negative). Silent local-map drift for negative-coord poses |
| Removed dead `if_first` | `SplineState.h` | Flag was set true in `init()` and never cleared. The `start_t_ns - dt_ns` branch in `minTimeNs()` was dead code |
| `getRCPs()` bounds check | `SplineState.h` | `t_knots[num_knot - 4 + i]` underflows for `num_knot < 4`. Safe today (no pruning) but a Phase 3.1 trap |
| Livox callback `points.empty()` check | `RESPLE.cpp` (3 callback variants) | Read `points[0]` after gating only on `point_num` field. Doesn't affect Ouster (we don't use Livox) |
| `wait_for(5s)` on map-update future | `RESPLE.cpp` `processData` | Plain `wait()` permanently locks the worker if lambda hangs. Now skips a cycle on timeout, logs ERROR, stays responsive |
| Worker iteration try/catch | `RESPLE.cpp` `processData` | Throw anywhere in IEKF / collect / deskew killed worker thread silently. Now logs + 50 ms sleep + continues |
| NaN/Inf input filter | `RESPLE.cpp` `mapIncremental` | NaN points fed to `Add_by_point` → `calc_dist` returns NaN → comparisons all false → tree corruption. Now skipped + WARN_THROTTLE |
| `Push_Down` race documentation | `ikd_Tree.cpp` | Inherited HKU-MARS race (parent-only lock when writing children's flags); documented with mitigation pointer (`num_threads=1`) |
| `ENABLE_DEBUG_O0` / `ENABLE_DEBUG_O1` / `ENABLE_DEBUG_O3G` build options | `CMakeLists.txt` | Diagnostic flags retained for future bug-chases; harmless when off |

### What's still open (status updated after the Phase 2–5 series)

Every item on the original 2026-05-01 list has since been resolved or
overtaken; kept here with outcomes so the bug-chase narrative stays complete:

- **Verify the Eigen-macro fix** — the production crash has not been
  re-observed since the `_GNU_SOURCE` / `EIGEN_MALLOC_ALREADY_ALIGNED=1` /
  `EIGEN_DEFAULT_ALIGN_BYTES=16` pinning landed, and the same dispatch
  mismatch later reproduced (and was fixed by the same pinning) in the
  standalone test build under the Phase 5 ASan CI job — independent
  confirmation of the diagnosis. A bag replay remains the final word.
- **Phase 2.3** (bounded input buffers) — capability landed (drop-oldest
  caps + counters; scan cap default-off per the decision gate).
- **Phase 3.x** — all four items landed (3.1 knot pruning, 3.2
  parameterized plane fit, 3.3 recovery policy, 3.4 radius pruning);
  only the tuning passes stay bag-gated.
- **Push_Down race** — fixed (Phase 2.4 per-node locking), and the
  related rebuild-path hazards fixed in Phase 2.5.
- **Operational safety net (Mapping stale window on RESPLE respawn)** —
  addressed by the respawn restart flow: `startCallBack` stages a restart
  consumed by the worker (Option B, PR #4), and `SplineState::init()` now
  also clears the knot deques (Phase 3.1 drive-by), so a respawn re-inits
  the window instead of querying a stale spline.

---

## Phase 2 — Concurrency hardening

**Status: 2.1 fixed (Phase 1.5 K1); 2.2 partially fixed (Phase 1.5 Fix C);
2.3 capability landed (drop-oldest caps + counters; scan cap default-off); 2.4 (Push_Down race) IMPLEMENTED + TSan-verified;
2.5 #2 (rebuild lock-order deadlock) FIXED + TSan-verified; 2.5 #1
(rebuild-vs-mutator data race) FIXED + TSan-verified (recursive whole-op
working_flag_mutex — see 2.5 below).**

Three independent items. 2.1 was preempted by Phase 1.5 (defensive shared
lock); 2.2's immediate race was closed by Phase 1.5 Fix C, full state-enum
refactor still optional; 2.3 unchanged. 2.4 landed once a ROS 2 + PCL toolchain
was available to compile- and TSan-verify it; verification surfaced two
pre-existing rebuild-path hazards now tracked as 2.5.

### 2.1 Verify ikd-Tree `Nearest_Search` lock contract

**Status: fixed defensively in Phase 1.5 (K1).** The racy fast-path was
removed; `Nearest_Search` now always takes `search_rw_mutex_` shared. The
gtest stress harness below is still useful as a regression test (Phase 5).

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

**Status: immediate race closed in Phase 1.5 (Fix C). Full enum refactor
optional.**

The bare `bool` reads/writes were the hazard; Phase 1.5 Fix C made them
`std::atomic<bool>`. The original three-bool design is still fragile to
reorder-by-future-edit, so the enum refactor below remains a desirable
clarity win — but it's no longer crash-relevant.

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

### 2.3 Bounded input buffers — **IMPLEMENTED (scan cap off by default)**

`pc_buff` (per-LiDAR) was unbounded; `imu_int_buff` had a HARDCODED
drop-oldest cap of 2000 (uncounted, unparameterized).

**Implemented (this commit):**
- `max_scan_buffer` (scans per LiDAR, default **0 = unbounded** — the
  decision gate asked for live evidence of growth under NORMAL operation,
  and the growth observed so far came from a deliberately overloaded
  sandbox worker, so the cap ships as an operational knob rather than a
  default). All 7 LiDAR callbacks push through one `pushScanBounded`
  helper: drop-oldest of `pc_buff`+`t_buff` in lock-step under `mtx_pc`.
- `max_imu_staging` (samples, default **2000** = the previously hardcoded
  value; 0 disables) replaces the magic constant in `getImuCallback`.
- Drops are counted (`dropped_scans_` / `dropped_imu_` atomics) and exposed
  in BOTH the `diagnostic_updater` output ("Scans/IMU Samples Dropped") and
  the Phase 4 `Diagnostics` message — non-zero means the caps are engaging
  (worker not keeping up with the sensors).

**Verification:** live injector run with deliberately tight caps
(`max_scan_buffer=2`, `max_imu_staging=300`, `num_threads=1`): staging depth
pins at exactly 300, drop counters advance (127 scans / 200 samples), and
the estimator stays healthy under the backpressure (NIS ~1–3, state OK,
IEKF at full rate) — graceful degradation instead of unbounded growth.

### 2.4 Fix the ikd-Tree `Push_Down` child-write race — **IMPLEMENTED + TSan-VERIFIED**

**Status: landed this commit. The design (below) was implemented verbatim and
compile-/TSan-verified once a ROS 2 + PCL toolchain became available. The
`num_threads=1` mitigation is retired.**

**Implementation (files `resple/include/ikd-Tree/ikd_Tree.{h,cpp}`):**
- `KD_TREE_NODE`: `TreeSize`, `invalid_point_num`, `down_del_num` →
  `std::atomic<int>`; the six deletion/propagation bools → `std::atomic<bool>`
  (`<atomic>` added). `working_flag` stays a plain bool (rebuild handshake,
  out of scope). Same-field-type assignments (`a = b` where both are atomic)
  use explicit `.load()` since `atomic = atomic` selects the deleted copy.
- `Push_Down`: fast lock-free pre-check; then PARENT `push_down_mutex_lock` for
  the body and the CHILD's lock around each child block (incl. the `*Rebuild_Ptr`
  branch, with `working_flag_mutex`/`Rebuild_Logger` nested inside the child
  lock → order node→working_flag, no inversion). `|=` sites became `if (x) y =
  true;`.
- Search call site simplified to a bare `Push_Down(root)`.
- Mutator flag-SET blocks in `Add_by_range`, `Delete_by_range`, and
  `run_operation`'s PUSH_DOWN replay wrapped in the node's `push_down_mutex_lock`.

**Verification performed:**
- Standalone compile of `ikd_Tree.cpp` (all three explicit instantiations:
  `PointXYZ`, `PointXYZI`, `PointXYZINormal`) clean under `-Wall -Wextra`.
- New gtest `resple/test/test_ikdtree_concurrency.cpp` — phased stress mirroring
  RESPLE's `mtx_map_` discipline (single-threaded delete/add arms a
  need_push_down frontier; a barrier then releases N readers that push it down
  concurrently). Functional run passes; the real gate is ThreadSanitizer.
- **Before/after under TSan** (same test, pre-2.4 tree vs this commit):
  reader-vs-reader `Push_Down` races on the node fields **16 → 0**. My new
  per-node mutexes appear in **no** lock-order cycle.
- See `resple/test/tsan_suppressions.txt` — it suppresses ONLY the two
  pre-existing, independent hazards now tracked as **2.5**, never `Search` /
  `Push_Down`, so a 2.4 regression still fails.

The full original design is retained below for reference.

---

**Original race (documented in the old `Push_Down` comment block):**

The race (documented in the `Push_Down` comment block, ikd_Tree.cpp ~1234):
`Push_Down(P)` writes the propagation fields of P's CHILDREN
(`tree_deleted`, `point_deleted`, `tree_downsample_deleted`,
`point_downsample_deleted`, `down_del_num`, `invalid_point_num`,
`need_push_down_to_*`) while holding only P's own `push_down_mutex_lock`
(taken at the Search call site, ikd_Tree.cpp ~1104-1117). A concurrent
`Search(C)` on the child holds C's mutex and reads/writes the same fields
of C → the two locks don't overlap → data race. Additionally, mutators set
`need_push_down_to_*` on subtree roots (sites at ~372, ~817, ~971) without
the node's mutex → a `Push_Down` clearing the parent flag can lose a
concurrent mutator's set (lost-update → deletions never propagate to that
subtree until the next delete touches the path).

**Key enabling observation:** every `KD_TREE_NODE` already carries a
`pthread_mutex_t push_down_mutex_lock` (initialized in `InitTreeNode`,
destroyed in `DeleteTreeNodes`) that is used at exactly ONE call site. The
per-node lock infrastructure for the proper fix already exists.

**Design (write-side mutual exclusion + read-side atomics):**
1. **Centralize locking inside `Push_Down`** and simplify the Search call
   site (~1104-1117) to a bare `Push_Down(root)` call:
   - fast pre-check (atomic loads, no lock): both `need_push_down_to_*`
     false → return;
   - lock the PARENT's `push_down_mutex_lock` for the body (serializes
     concurrent `Push_Down(P)` instances and protects P's flag clears);
   - in each child block, additionally lock the CHILD's
     `push_down_mutex_lock` around the child-field writes (including the
     `*Rebuild_Ptr` branch, which keeps its existing `working_flag_mutex` +
     `Rebuild_Logger` handling). Lock order is strictly parent→child along
     tree edges → acyclic → deadlock-free. A thread holds at most two node
     mutexes at a time.
2. **Wrap the three mutator flag-set sites** (~372, ~817, ~971) with that
   node's `push_down_mutex_lock`. Contract afterwards: *a node's deletion-
   propagation fields are written only while holding that node's
   `push_down_mutex_lock`; reads are lock-free atomics.* This closes the
   lost-update window by mutual exclusion (no ordering subtleties left).
3. **Convert the racy fields to atomics** so the lock-free reads in
   `Search`/`Box_Search`/`Radius_Search`/`Criterion_Check`/`Update` are
   defined behavior (TSan-clean): the six bools → `std::atomic<bool>`,
   `down_del_num`/`invalid_point_num`/`TreeSize` → `std::atomic<int>`.
   Mechanical notes from the site survey:
   - `std::atomic<bool>` has no `|=` — the 12 `x |= y` sites (4 in
     Push_Down blocks ×2 sides + ~362/363) become `if (y) x = true;`
     (safe under the new per-node lock);
   - no `KD_TREE_NODE` is copied by value and no flag/counter has its
     address taken (verified by grep), so atomics are drop-in elsewhere;
   - the two `memset(&range, ...)` calls (~116/133) touch only the local
     `BoxPointType`, not nodes — no change needed;
   - in-class initializers (`= false`, `= 0`, `= 1`) remain valid for
     atomics.
4. **Retire the mitigation:** remove the `num_threads=1` guidance from the
   `Push_Down` comment and CLAUDE.md hazards table; multi-threaded k-NN
   (OpenMP `findCorresp`) becomes safe by contract.

**Verification:** colcon build; unit suite; the §2.1 gtest stress harness
(N parallel `Nearest_Search` + concurrent `Add_Points`/`Delete_Point_Boxes`)
under TSan — before/after comparison should show the Push_Down reports gone;
bag replay (`scripts/run_sanitizer_replay.sh tsan <bag>`) as the end-to-end
gate. Benchmark a bag with `num_threads=8` vs `=1` to quantify the recovered
parallelism.

**Complexity: medium-high** (mechanical breadth, not conceptual depth). The
per-node mutex in the hot search path costs one uncontended pthread lock per
node visited only when `need_push_down_to_*` is set (the pre-check skips the
lock otherwise) — deletions are bursty (FOV segment + downsample), so the
common search path stays lock-free.

### 2.5 Rebuild-path hazards surfaced by the 2.4 stress test — **#1 FIXED, #2 FIXED**

**Status: found while TSan-verifying 2.4; both pre-existing (present with AND
without the 2.4 change) and inherited from upstream HKU-MARS. #1 (rebuild-vs-
mutator data race) remains deferred and suppressed in
`resple/test/tsan_suppressions.txt`. #2 (the lock-order inversion / deadlock)
is FIXED this commit.**

1. **(FIXED — recursive whole-op `working_flag_mutex`, see below) Rebuild-thread vs. mutator data race on `KD_TREE_NODE` fields.**
   `multi_thread_rebuild` reads `(*Rebuild_Ptr)->father_ptr` (`ikd_Tree.cpp`
   ~255) while a concurrent mutator's `Update` writes `father_ptr`/aggregate
   fields (~1564), and the rebuild swap's ancestor `Update`-walk touches nodes
   the mutator traverses. Vanilla ikd-Tree has no `search_rw_mutex_` and races
   here identically, so this is the upstream baseline. Functionally benign in
   testing (search results stay consistent, `err=0`) but a real race; the
   production-faithful `test_ikdtree_concurrency` does not trigger it, only the
   aggressive focused stress does. Tracked as a follow-up.

2. **(FIXED) `search_rw_mutex_` ↔ `working_flag_mutex` lock-order inversion
   (real deadlock).**

   **Root cause (corrected from the first diagnosis):** the Phase 1.5 K1
   extension wrapped the *mutating* fast-path calls in `Add_Points` /
   `Add_Point_Boxes` / `Delete_Points` / `Delete_Point_Boxes` /
   `Delete_by_range`-downsample in `search_rw_mutex_` (shared). Those calls take
   `working_flag_mutex` internally when the recursion reaches a *deeper*
   `*Rebuild_Ptr` boundary. The fast-path guard only checked the *top-level*
   `*Rebuild_Ptr != Root_Node`, so a deeper rebuild target still led to
   `working_flag_mutex` being taken **while holding `search_rw_mutex_` shared**.
   The background rebuild thread holds `working_flag_mutex` and takes
   `search_rw_mutex_` **unique** (`multi_thread_rebuild` ~258/292) — the
   opposite order → a real cycle. TSan-confirmed; observed to actually HANG the
   committed test under load (the original reason that test carries a `TIMEOUT`).
   Reachable in production: `mapIncremental`/`lasermapFovSegment` mutate while
   the kd-tree's background rebuild thread runs.

   **Fix:** remove the `search_rw_mutex_` shared lock from the five *mutating*
   fast-path blocks. This reverts the mutators to the upstream ikd-Tree
   coordination, where a mutator self-synchronises against the rebuild thread
   via `working_flag_mutex` at the rebuild boundary (which also protects against
   the swap + node free) — so the extra shared lock was never needed there. K1's
   shared lock is KEPT where it is correct and load-bearing: the *search*
   functions (`Nearest_Search`/`Box_Search`/`Radius_Search`) and the one genuine
   lock-free *read* in `Add_Points` (`Search_by_range(Root_Node, …)`), which have
   no `working_flag` coordination of their own. See the rewritten lock-discipline
   comment at the top of `Add_Points`.

   **Verification:** `test_ikdtree_concurrency` under TSan — before: hangs /
   reports the inversion; after: completes, 0 lock-order-inversions, 0 data
   races (production-faithful discipline), test passes. The aggressive focused
   stress still shows the #1 rebuild-vs-mutator races (suppressed), but no
   inversion. The `deadlock:` suppressions were removed so a regression fails.

   **Trade-off noted honestly:** the K1 shared lock had been incidentally
   *masking* the #1 rebuild-vs-mutator races (by excluding the mutator from the
   rebuild thread's `search_rw`-unique sections). Removing it re-exposes #1 under
   aggressive stress. We accept this: a deadlock (unrecoverable hang) is a
   strictly worse failure mode than the upstream-baseline #1 race (benign in
   testing), and #1 was already the deferred upstream behavior. Fully suppressing
   #1 as well would need the larger redesign below.

**#1 FIX (landed):** the recursive-`working_flag_mutex` option, chosen after a
new aggressive stress (`IkdTreeConcurrency.RebuildVsMutatorRace`, production
`mtx_map_` discipline + bursty deletes hammering multi-thread rebuilds) showed
the raced field set is wider than first diagnosed — the mutator pull-up
`Update()` rewrites the node range float-arrays (memcpy), `alpha_*` and child
links as well as `father_ptr`, so field-level atomics could not cover it.

- `working_flag_mutex` is now initialized `PTHREAD_MUTEX_RECURSIVE`
  (`start_thread`), and the four public mutators (`Add_Points`,
  `Add_Point_Boxes`, `Delete_Points`, `Delete_Point_Boxes`) hold it across
  their WHOLE operation (`ScopedPthreadLock` RAII); their internal rebuild-
  boundary acquisitions nest on the recursive mutex.
- This gives the mutator and the rebuild thread mutual exclusion over the
  shared ancestor region (pull-up writes vs the rebuild's `father_ptr` read +
  swap ancestor `Update`-walk) that previously raced with no common lock.
- Lock-order safety: order stays `working_flag → {search_rw, rebuild_logger,
  per-node push_down}` on both sides; `Rebuild()` only TRYlocks
  `rebuild_ptr_mutex_lock`, so the opposite-order pairing with
  `multi_thread_rebuild` cannot deadlock — a contended trylock defers the
  rebuild request to a later mutation.
- Cost: a mutator can now also block on the rebuild thread's flatten/swap
  sections when operating on a DISJOINT subtree (previously only boundary-
  touching ops blocked). Production has one map mutator (the async map task)
  and the live sweep shows an unchanged IEKF rate.

**Verification:** measured during the fix session with back-to-back runs of
identical binaries differing only in `ikd_Tree.cpp`: pre-fix the new stress
reported ~200 TSan races (`multi_thread_rebuild`/`Update`/memcpy on node
fields), post-fix 0; BOTH concurrency tests run TSan-clean **with the
suppressions file emptied** (the `race:multi_thread_rebuild` /
`race:multi_thread_ptr` entries are deleted, so any regression now fails the
gate). Live data-path TSan sweep: clean, IEKF rate unchanged, clean shutdown.

**Detection-power caveat (be honest with yourself when re-running):** the
pre-fix window is deep in scheduler territory — it fired reliably only on a
heavily loaded host (CPU oversubscription stretches the rebuild thread's
flatten/swap windows into the mutator's bursts); several pre-fix runs on a
lightly loaded box reported nothing. The committed stress is therefore a
CANARY: any TSan report is a real regression and fails CI
(`halt_on_error=1`), but a quiet run is necessary, not sufficient. The
4-vCPU CI runner (4 spin readers + mutator + rebuild thread = oversubscribed)
is exactly where detection chances are best. See the tuning note in the test
itself before "optimizing" its configuration — throttled readers and
full-extent mutation were each measured to make it fast but blind. Bag
replay remains worthwhile as an end-to-end confidence pass when a bag is
available.

---

## Phase 2.6 — Live data-path TSan sweep (synthetic injection) — **2 races FIXED**

**Status: the Phase 0 sanitizer sweep, finally run against the LIVE data path.**

The earlier lifecycle-only sanitizer runs (TSan + ASan, clean) never exercised
the data-processing concurrency because no LiDAR/IMU data flowed. With the ROS
Python stack unusable in the sandbox (`rclpy`'s `_rclpy_pybind11` C extension is
absent, so `ros2`/`ros2 bag`/`launch_test` don't run), a small **C++ rclcpp
injector** drives the node instead: it publishes static TF + stationary IMU
(100 Hz) + a static "room" `PointCloud2` (10 Hz) with zero motion, so the
estimator gravity-inits, tracks at the origin, and the worker runs its full
pipeline (deskew → `findCorresp` parallel k-NN → IEKF → `mapIncremental`
`Add_Points` → `lasermapFovSegment` `Delete_Point_Boxes` → spline growth).

**Method note (important for anyone repeating this):** with the default
`num_threads=4`, TSan reports ~253 races, but the vast majority are **GCC
`libgomp` false positives** — TSan does not model libgomp's parallel-for
barriers, so it flags benign accesses across the implicit barrier between IEKF
phases. Re-running with `num_threads=1` (OpenMP serialized) collapses that to
**18 genuine cross-thread races**, which are the real signal. (A bag replay
should use the same `num_threads=1` trick, or a TSan-aware OpenMP runtime.)

The 18 reduced to two distinct real bugs, both now fixed (TSan before/after:
18 → 0 with `num_threads=1`, the known 2.5 #1 rebuild race suppressed):

1. **Lidar input-buffer `empty()` race (`RESPLE.cpp` ~549).** `processData`
   drained the per-LiDAR buffer with `while (!lidar_data.t_buff.empty())` — the
   `empty()` read was OUTSIDE `mtx_pc`, racing the sensor callback's locked
   `t_buff.push_back` on the deque internals. Exactly the bug class Phase 1.5
   Fix C closed for the IMU `imu_int_buff`, but on the LiDAR path. **Fix:** check
   emptiness under `mtx_pc` (`while (true) { lock; if (empty) break; … }`).

2. **Spline read in the async map task without `spline_mutex_`
   (`RESPLE.cpp` `lasermapFovSegment`).** The async map-update task read the
   spline via `getPositionLiDAR → itpPosition/itpQuaternion` holding only
   `mtx_map_` (unique), while the worker GROWS the spline via
   `collectMeasurements → addOneStateKnot` under `spline_mutex_` **alone** (no
   `mtx_map_`). So `mtx_map_` unique did NOT exclude that write — the unlocked
   read raced with it. The `getPositionLiDAR` "race-free" comment only accounted
   for the IEKF spline *reads* (which hold `mtx_map_` shared), not the
   `collectMeasurements` spline *writes*. **Fix:** take `spline_mutex_` around
   the spline reads in `lasermapFovSegment`; lock order `mtx_map_ → spline_mutex_`
   is preserved (the async task already holds `mtx_map_` unique, and the worker's
   spline-growth path holds `spline_mutex_` alone, so no cycle).

**Verification:** `colcon build` (TSan and RelWithDebInfo) green; the synthetic
injector reaches steady state (IEKF running, spline growing); TSan real-race
count 18 → 0. The sweep is reproducible via `./scripts/run_data_sweep.sh`
(injector package + config under `resple/test/tools/`). For a
production-confidence pass, replay a real bag through the same TSan build with
`num_threads=1`.

**Also confirmed (not bugs):** the lifecycle/shutdown paths are TSan- AND
ASan/UBSan-clean; and `Spline Knots` grows unboundedly under zero pruning
(hazard 4 / Phase 3.1 — expected, measured live here for the first time).

---

## Phase 2.7 — Bounded-join detach race (found during the 3.1 sweep)

**Status: FIXED (both nodes), found by a TSan runtime CHECK-abort at shutdown
while verifying Phase 3.1.**

`joinProcessingThreadBounded` (RESPLE + Mapping) dispatched `join()` onto a
`std::async` task and, on timeout, called `detach()` from the lifecycle
thread — two threads operating on the SAME `std::thread` object. When the
worker exited right at the deadline (TSan's slowdown made this reproducible),
the async `join()` consumed the thread id concurrently with the `detach()` →
UB; TSan's runtime aborted with `ThreadRegistry::ConsumeThreadUserId CHECK
failed` inside `pthread_detach`. The async future's destructor also blocks
until `join()` returns, so on a genuinely wedged worker the "bounded" join
was never bounded.

**Fix:** the worker lambda's final action is a release-store to
`processing_thread_exited_`; the bounded join polls that flag against the
deadline. Flag set → plain `join()` (guaranteed prompt); deadline hit →
`detach()` — in both cases exactly one thread ever touches
`processing_thread_`. Verified: the Phase 3.1 TSan sweep (30 s synthetic
injection + SIGINT shutdown) completes with zero TSan output of any kind.

---

## Phase 3 — Spline / mapping accuracy

**Status: pending Phase 0 data.**

Four items. Each is a distinct PR with a before/after trajectory-plot
regression before merging.

### 3.1 Sliding-window knot pruning — **IMPLEMENTED**

**Status: landed this commit. The decision gate was satisfied by the Phase 2.6
live data-path sweep, which measured `Spline Knots` growing unboundedly (one
knot per `knot_hz` tick, no pruning) for the first time on a running node.**

`SplineState::t_knots` / `q_knots` / `ort_delta` grew indefinitely in BOTH
nodes (RESPLE's estimator spline and Mapping's `spline_active_`).

**Implementation (`SplineState.h` + both nodes):**
- `SplineState::pruneFrontKnots(keep_knots)` drops the oldest knots and slides
  the 3-slot idle window into their place. Key invariant that makes this safe:
  the idles act as knots `-3..-1` (`t_idle[j]`/`q_idle[j]` hold the pose of
  knot `j-3`, `ort_delta_idle[j]` the delta arriving at it — the convention
  `prepareInterpolation`/`itpPose`/`itpQuaternion` read them with), so after
  pruning K knots the old knots `[K-3..K-1]` become the new idles and
  interpolation over the retained range is **bit-for-bit identical** (unit-
  tested as an exact-equality property, not approximate).
- Absolute knot indexing is preserved via `num_knots_pruned_`:
  `totalKnots() = numKnots() + pruned` is the monotonic index space of the
  est_window protocol. `getSplineMsg` emits absolute `start_idx`; `updateKnots`
  translates `start_i` through the receiver's own pruned offset (a window
  entirely before the prune point is a no-op); a hard floor of 8 retained
  knots protects `getRCPs`/`updateRCPs` (last 4) and the message window
  (last 5). `start_t_ns` advances in lock-step so time-based queries need no
  translation, and the Phase 1.5 B clamps now clamp to the pruned
  `minTimeNs()`.
- **RESPLE node:** prunes each worker cycle, under `spline_mutex_`, AFTER the
  est_window publish (publish-then-prune keeps every knot on the wire before
  it can drop). The publish gate and `getSplineMsg` hint now use
  `totalKnots()` — a `numKnots()` gate would stop firing once pruning caps
  the retained count. Diagnostics gain `Spline Knots (total incl. pruned)` so
  the growth signal stays observable.
- **Mapping node:** prunes `spline_active_` on the worker after the
  path/odom/control-point publishes; the publish gate likewise moved to
  `totalKnots()`. `updateKnots`' idle-copy is additionally gated on
  `num_knots_pruned_ == 0` so a stale/duplicate window can no longer clobber
  the slid idles.
- **Parameter:** `spline_prune_keep_knots` (both nodes, default **600** ≈ 6 s
  at `knot_hz=100`; `0` disables; values 1–99 clamp to 100 with a WARN). 600
  is far wider than every backward-looking consumer: IEKF last 4 knots,
  est_window last 5, deskew ~10 (one scan), Mapping's path/scan lag.
- **Drive-by fix:** `SplineState::init()` now clears the knot deques — a
  re-init (Mapping consuming a RESPLE respawn) previously reset `num_knot`
  but left the deques populated, so knot index i silently read the PREVIOUS
  run's data once knots were re-added.

**Verification:** `resple/test/test_spline_state.cpp` — 10 new ROS-free unit
tests (exact interpolation equivalence incl. `itpPose` Jacobians, small-prune
idle reuse, repeated-prune equivalence, floor/no-op edges, re-init reset, RCP
round-trip, and a sender/receiver est_window protocol simulation with pruning
on both sides plus a stale-window redelivery case). The standalone ROS-free
build compiles `SplineState.h` against field-compatible stubs of the
`estimate_msgs` headers (`test/stubs/`); the colcon `BUILD_TESTING` path uses
the real generated messages.

### 3.2 Plane-fit hardening — **IMPLEMENTED (defaults preserve legacy behaviour)**

**Status: landed this commit.** `Association::findCorresp` previously used a
hardcoded k-NN distance gate (`pointSearchSqDis < 5` / search radius 2.236)
and a hardcoded `esti_plane` threshold `0.1f`, with no degeneracy rejection
and no visibility into where candidates were dropped.

**Implemented:**
- **`Association::CorrespConfig`** (plumbed RESPLE node → `Estimator`
  member, like `n_iter` → both `findCorresp` paths, CPU and CUDA):
  - `nn_max_sq_dist` (param, default 5.0) — the k-th-neighbor squared
    distance gate; the `Nearest_Search` radius is derived as its sqrt. NOTE:
    the CUDA k-NN early-termination shell was validated for the default
    2.236 m gate; raising above ~5.76 m² under `ENABLE_CUDA` needs that scan
    widened first (comment at the field).
  - `plane_fit_thresh` (param, default 0.1) — esti_plane residual threshold.
  - `plane_min_cond_ratio` (param, default 0 = off) — degeneracy guard:
    forwarded through `esti_plane` to `resple::geom::fitPlane`'s
    rank-revealing-QR pivot-ratio test (the §3.2 hook landed with the
    geometry core, unit-tested there). Rejects collinear / rank-deficient
    neighbor patches that lie on infinitely many planes. Deliberately OFF by
    default — turning it on changes which correspondences feed the IEKF, so
    it stays a tuning decision gated on a known-good-bag benchmark.
- **Per-update funnel counters** (`Association::CorrespStats`, accumulated by
  the worker into cumulative atomics): `candidates → passed_window →
  passed_distance (full k-NN within gate) → passed_plane (esti_plane) →
  used_in_IEKF`. Published as per-window deltas in diagnostics ("Corresp
  …(last window)"). Stage-to-stage drops localize losses: out-of-window vs
  sparse map vs degenerate/noisy patch vs association outlier.

**Verification:** colcon build + full test suites green (defaults are
bit-for-bit the legacy values: same gate constants, min_cond_ratio off);
live TSan data-path sweep clean with the counters active in the OpenMP
parallel region (thread-local tallies, one atomic merge per thread).

**Remaining (tuning, needs bags):** choose a production `plane_min_cond_ratio`
(and revisit `nn_max_sq_dist` per sensor) — full procedure and decision gate
in §6.3.

### 3.3 Divergence detection and recovery — **DETECTION DONE**

After each IEKF pass, the filter's health was invisible externally. Bad
posterior covariance (collapse or NaN) propagated into published odometry
and downstream consumers without any warning.

**Implemented (detection + IMU input health):**
- `Estimator::update()` now computes the measurement-space NIS
  `nu^T (H P H^T + R)^{-1} nu` in both branches (directly from the factored
  `S` in the small-measurement branch; via the Woodbury identity reusing the
  existing Cholesky factor in the information-form branch). A failed/skipped
  update reports NIS = NaN. Exposed via `lastNis()` / `lastNisDof()`.
- `utils/filter_health.h::NisDivergenceDetector` (unit-tested, ROS-free):
  windowed NIS mean vs. dof with WARN / DIVERGED thresholds, consecutive-
  breach counting, and hysteresis recovery. Fed by the worker each IEKF
  cycle; verdict + window mean exported to diagnostics ("Filter Consistency
  (NIS)"), with a throttled ERROR log on DIVERGED. Tunables:
  `nis_window` (32), `nis_warn_ratio` (2.0), `nis_diverged_ratio` (4.0),
  `nis_breach_limit` (3).
- `utils/filter_health.h::ImuHealthMonitor` (unit-tested, ROS-free): per-
  sample NaN / saturation / stuck-sensor / time-jump faults plus windowed
  stationary noise-floor and bias estimates — aimed at poor built-in IMUs
  (e.g. Ouster). Fed in `getImuCallback` (with `acc_ratio` scaling applied);
  fault bits + stationary gyro-bias norm exported to diagnostics, throttled
  WARN on faults.

**Recovery policy — IMPLEMENTED (parameter-selected, default off):**

Per the operator decision (2026-06-10), both policies are implemented behind
`nis_recovery_mode` (default `off` = detection-only legacy behaviour):

- **`hold`** — on DIVERGED, suspend odometry/TF publication (the last
  published pose is the held estimate; the downstream odom EKF coasts on its
  other sources instead of fusing an untrustworthy pose). The hold latches
  and releases only on a full OK verdict, so WARN during the detector's
  hysteresis recovery keeps the gate closed. est_window / map updates
  continue so internal estimation keeps running and the detector can observe
  recovery. Logged on entry/exit; `NIS Recovery Hold Active` in diagnostics.
- **`reset`** — on DIVERGED, reinflate the IEKF covariance to the
  configure-time prior (`Estimator::resetCovarianceToPrior`), keeping the
  state (spline + biases): an over-confident covariance is exactly what the
  NIS verdict detects, and reinflation lets new measurements re-correct the
  state. The detector window restarts so the next verdict reflects the
  post-reset filter. `NIS Recovery Resets (cumulative)` in diagnostics.

Note: `reset` deliberately reinflates covariance rather than rewinding the
state to a stored "last good pose" — a state rewind would need pose history
plus a spline/Mapping window restart (respawn-equivalent), and the
covariance reinflation addresses the failure mode NIS actually measures.

**Remaining (optional):**
- Covariance trace / `λ_min` metrics on a dedicated status topic
  (the NIS verdict largely subsumes them for divergence purposes).
- Policy validation on a bag with a real divergence episode (hold-vs-reset
  comparison is a tuning exercise, like the §3.2 thresholds).

### 3.4 Map pruning policy — **RADIUS PRUNING IMPLEMENTED (off by default)**

`lasermapFovSegment` uses a sliding cube around the current pose (size set by
`cube_len`). The cube only deletes when the pose nears its edge — wandering
inside a large cube (the shipping `cube_len` is 1000 m) keeps stale far
geometry alive for the whole run.

**Implemented (this commit):**
- `RESPLE::pruneMapRadius` — when `map_prune_radius > 0`, keep only the
  axis-aligned box of half-extent R centered on the current lidar pose: on
  first activation, and whenever the pose has moved > 10% of R since the last
  prune, delete the slabs of the local-map cube outside the retention box.
  Runs inside the async map task under `mtx_map_` unique (same locking as the
  cube deletions), every tick of `lasermapFovSegment` with its own movement
  hysteresis.
- R is floored at **2× `det_range`** so pruning can never bite into the
  sensor's live measurement sphere (all findCorresp matches are within
  `det_range`); a configure-time WARN reports the effective radius.
- The cube∖box slab decomposition lives in the geometry core
  (`resple::geom::subtractBox`, ≤6 disjoint slabs) with unit tests proving
  disjointness + exact tiling by dense sampling.
- Diagnostics: `Map Radius Prunes (cumulative)`.
- **Default `map_prune_radius = 0` (disabled)** — legacy cube-only behaviour.
  Enabling it is an operational choice per environment (e.g. long tunnel
  traverses vs loopy sites where revisited geometry is useful).

**Deliberately not done:** temporal decay (per-cell timestamps — storage cost;
deferred, as originally planned).

---

## Phase 4 — Diagnostics publisher

**Status: IMPLEMENTED (this commit).**

`estimate_msgs/msg/Diagnostics` — typed fields (plottable directly in
Foxglove / PlotJuggler, unlike the string-keyed `diagnostic_updater` output,
which remains on `/diagnostics` for aggregation). Published by the RESPLE
node on the relative topic **`resple_diagnostics`** (the production namespace
yields `/localization/resple/resple_diagnostics` — the proposal's
`…/diagnostics` literal would collide with the global `diagnostic_updater`
topic on an un-namespaced node), once per processed worker frame (~20 Hz),
best-effort QoS.

Carries everything from the proposal table below: knot count + monotonic
total, the three input-buffer depths, IEKF failures / NIS / dof / filter
state, the §3.3 recovery state, pose-covariance trace + λ_min (6×6 at
`maxTimeNs`), the §3.2 correspondence funnel, deskew out-of-range count,
ikd-tree size, §3.4 prune count, and per-stage timings for the last frame
(drain / IEKF / deskew / frame total / async map update — measured live, the
map-update duration from inside the async lambda via a relaxed atomic).

Original proposal kept below for reference.

Today, diagnostics scatter across `diagnostic_updater` fields and ROS logger
output. Consolidate into a single topic for easier monitoring.

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

---

## Phase 5 — Regression tests

**Status: DONE except the bag-gated smoke test.**

| Test | Scope | Status |
| --- | --- | --- |
| `SplineState` unit | Bounds, bootstrap, pruning | **done** (Phase 3.1: `test_spline_state.cpp`, 10 tests incl. exact prune-equivalence + est_window protocol) |
| `Association::findCorresp` stress | Multi-threaded `Nearest_Search` under concurrent mutation | **done** (`test_ikdtree_concurrency.cpp`: phased Push_Down test + `RebuildVsMutatorRace` canary) |
| `Estimator<>` unit | Joseph-form PSD property | **done** (geometry core: `JosephUpdate.ResultIsSymmetricAndPSD` — the in-Estimator update delegates to it) |
| Bag-replay CI smoke | Replay short bag, pose tolerance, divergence flag | **pending a representative bag — design ready in §6.2** |
| Sanitizer CI | TSan + ASan gates on every push/PR | **done (this commit)** — see below |

### What landed (this commit)

- **`.github/workflows/unit-tests.yml`** now has three jobs:
  - `estimator-core` — the ROS-free suite (also gained the previously missing
    `libboost-math-dev`, which `test_spline_state` → `math_tools.h` needs —
    CI would have broken on the Phase 3.1 commit otherwise);
  - `estimator-core-asan` — the same suite under ASan+UBSan with
    `detect_leaks=1` (no PCL installed, so the long ikd-Tree stress is
    excluded there by the existing PCL gate);
  - `ikdtree-tsan` — both ikd-Tree concurrency tests under ThreadSanitizer,
    run directly (no ctest timeout) with `halt_on_error=1` and the (empty)
    suppressions file, generous job timeout.
- **The new ASan job immediately paid for itself**, catching two real issues
  in the standalone test build:
  1. the **Phase 1.6 Eigen allocator mismatch** (heap-buffer-overflow in
     `handmade_aligned_free`) — the package build pins
     `_GNU_SOURCE`/`EIGEN_MALLOC_ALREADY_ALIGNED=1`/`EIGEN_DEFAULT_ALIGN_BYTES=16`
     but the standalone test build never did; it now applies the same pinning;
  2. a **1-node leak per `KD_TREE::Build()`** — upstream allocates
     `STATIC_ROOT_NODE` and never frees it; the destructor (and a re-`Build`)
     now release it.
- ctest timeouts for the concurrency binary raised 120 → 300 s (native
  runtime of the stress is ~10 s on a many-core box but ~70 s on 4 cores).

The old wire-up note (flip `BUILD_TESTING` in `colcon_defaults.yaml`) is
overtaken: the package's colcon `BUILD_TESTING` path has been live since the
Phase 3.1 commit (all unit tests run under `colcon test`).

---

## Phase 6 — Bag-gated work: implementation-ready designs

**Status: DESIGNED, blocked only on a representative recorded dataset.**
Everything else in this plan is implemented. The four items below are the
remaining work; each is written so a future session can execute it without
re-deriving the approach (same convention as the §2.4 design that preceded
its implementation). Items 6.1–6.2 are validation passes; 6.3–6.4 produce
parameter/policy decisions.

**Shared prerequisite:** one or more representative bags. Minimum viable:
a 3–10 min sequence with the production sensor (Ouster OS1-16 + built-in
IMU) covering normal motion. Ideal additions: a geometrically degenerate
stretch (long corridor/tunnel) for 6.3/6.4, and ground truth (or a trusted
reference trajectory) for 6.3. The upstream dataset configs
(`config_eee02.yaml` etc.) are usable stand-ins for everything except
production-faithful tuning.

### 6.1 Sanitizer bag replay (closes the original Phase 0 task)

Goal: end-to-end memory/thread-safety confidence on REAL data — the live
synthetic sweep (`run_data_sweep.sh`) covers the pipeline shape but not real
timing, dropout, or point-distribution patterns.

Steps:
1. `./scripts/run_sanitizer_replay.sh tsan <bag>` with `num_threads: 1` in
   the config (libgomp barriers are TSan false positives; the data-sweep
   script's sed trick shows how). Save stderr.
2. Same with `asan`. ASan needs no thread cap.
3. Pass criteria: zero TSan reports (suppressions file is empty and must
   stay so), zero ASan/LSan reports, clean SIGINT shutdown, and the
   `resple_diagnostics` stream showing: knot plateau at
   `spline_prune_keep_knots`, `dropped_* == 0` (caps default-off),
   `deskew_out_of_range == 0`, `iekf_numerical_failures == 0`.
4. Deliverable: paste the run summary into this file under a dated note;
   if anything fires, the stack IS the bug — file it as a new hazard row.

#### 6.1 RESULTS — TSan leg, 2026-06-10 (HelmDyn01, LIO)

**Environment.** `resple_ws` docker image (`osrf/ros:jazzy-desktop`, default
`rmw_fastrtps_cpp`); RESPLE + Mapping driven headless (no rviz) by the new
`scripts/sanitizer_replay.sh tsan`; full HelmDyn01 bag (205 s, Livox Mid360,
`/livox/lidar`+`/livox/imu`) replayed at rate 1.0; config forced to LIO
(`if_lidar_only:false`) + `num_threads:1` (+`OMP_NUM_THREADS=1`) so libgomp
parallel-for barriers don't masquerade as races. Empty RESPLE-code suppressions.

**This was the first time LIO + the Mapping node ran under TSan on a real bag**
(prior TSan work used the synthetic injector + unit tests, which drive the
RESPLE node's data path only). It immediately surfaced **~33 data races + 1
lock-order inversion**, all genuine (num_threads=1). Iterative fix+re-run cycles
took the RESPLE-code race count to **zero** — the final run is fully CLEAN (no
TSan reports at all, with only the justified Fast-DDS suppression active). Note
the scheduler-dependence: bug 43 (`getEstCallback`) only surfaced *after* the
Fast-DDS noise was suppressed, on a later run — a reminder that a quiet TSan run
is necessary but not sufficient (see the §2.5 detection-power caveat). Fixes
(all in the `resple` submodule):

| # | Bug (TSan) | Root cause | Fix | File |
| --- | --- | --- | --- | --- |
| 39 | Mapping lidar callback re-entrant on shared scratch (~25 reports: `livoxLidarCallback`/`pcl::Filter::filter`/`transformPointCloud`) | The 7 sensor callbacks mutate inherited member scratch (`pc_last`/`pc_last_ds`/`ds_filter_each_scan`/transform state) assuming single-threaded dispatch; under the MultiThreadedExecutor TSan sees no happens-before between consecutive dispatches of the MutuallyExclusive group | Added `MappingBase::cb_mtx_`, locked at the top of every sensor callback — makes the serialization explicit and TSan-visible | `Mapping.cpp` |
| 40 | `Mapping::process()` lock-order inversion (potential deadlock) | Swap branch took `m_spline`→`maps`; publish/prune branch took `maps` (`ScopedMappingsLock`)→`m_spline` — cyclic order | Hoisted `m_spline` before `ScopedMappingsLock` in the publish branch → always `m_spline`→`maps` | `Mapping.cpp` |
| 41 | `MappingBase::processScan` reads `pc_L_buff.size()` outside `mtx` (worker vs callback `push_back`) | The throttled diagnostic log read the deque depth after the locked pull scope closed | Read the size under `mtx` | `Mapping.cpp` |
| 42 | `KD_TREE::size()` lock-free read of non-atomic `Root_Node` vs rebuild swap | `size()` kept the old `Rebuild_Ptr`-check fast-path that K1/§2.5 removed from the *search* functions but never applied to this *reader*; called from the worker's diagnostics (`RESPLE.cpp:1085 dmsg.map_size`) without `mtx_map_` | Take `search_rw_mutex_` shared (the K1 idiom); `TreeSize` is already atomic | `ikd-Tree/ikd_Tree.cpp` |
| 43 | `Mapping::getEstCallback` re-entrant on `getEstCallback_logged_` + `spline_pending_` staging | The `/est_window` `Estimate` subscription is on the node default group; same MutuallyExclusive-but-TSan-invisible-HB issue as the lidar callbacks. Surfaced only after the Fast-DDS noise was suppressed (scheduler-dependent) | Added `est_cb_mtx_`, locked across `getEstCallback` | `Mapping.cpp` |

**Residual (NOT RESPLE code): 4 Fast-DDS races.** The only reports left (2 per
node) are inside eProsima Fast-DDS — `fastcdr::Cdr::serialize_array` (publish)
vs `deserialize_array` (receive/statistics) over a `fastrtps TopicPayloadPool` /
`StatisticsListenersImpl` buffer, i.e. the middleware racing its own payload
pool with zero RESPLE involvement. **The deployment uses `rmw_zenoh`, not
Fast-DDS**, so this path does not exist in production; it is a property of the
container/CI default RMW. Suppressed with justification in
`resple/test/tsan_suppressions.txt` (`race:…Cdr::serialize_array` /
`deserialize_array`) so the gate measures RESPLE code, not the DDS vendor.

**Caveats / still open.**
- Under TSan at rate 1.0 the instrumented worker backlogs (`pc_buff`~2.4–2.7k).
  This maximizes callback-vs-worker detection but isn't normal timing; the four
  fixes are structural (a missing lock, an inconsistent lock order, a lock-free
  reader) so they hold regardless. A reduced-rate confirming run is worthwhile.
- **Deployment-faithful re-validation under `rmw_zenoh`** is the proper close
  (would also confirm the Fast-DDS residual vanishes). Pending
  `ros-jazzy-rmw-zenoh-cpp` in the image + a `rmw_zenohd` router.
- CLAUDE.md's hazard table should gain rows 39–43 + 44–45 (mirrors this section).

#### 6.1 RESULTS — ASan/UBSan leg, 2026-06-10 (HelmDyn01, LIO)

Same harness, `scripts/sanitizer_replay.sh asan` (ENABLE_ASAN = ASan+UBSan,
`detect_leaks=1`, LIO, full bag). **RESULT: CLEAN** — zero ASan/LSan/UBSan
reports on the real data path; node healthy (LIO gravity init, knot plateau at
600, IEKF running, no DIVERGED, clean shutdown). Confirms the §6.1 fixes +
the DDS-robustness hardening below compile and introduce no memory errors.

#### 6.1 — DDS/RMW data-handling robustness hardening (2026-06-10)

Operator requirement: the data path must tolerate DDS/RMW faults (torn /
partial / corrupt / concurrently-delivered messages) — degrade gracefully, never
crash or corrupt. We cannot fix races *inside* the middleware (the suppressed
Fast-DDS payload-pool race), but we can ensure our ingest validates everything.
Audit of all ingest paths in both nodes:

| # | Gap | Fix |
| --- | --- | --- |
| 44 | **Mapping callbacks had no top-level try/catch** (7 lidar + `getEstCallback`) — a throw (bad_alloc, PCL/Eigen) escaped the executor → `std::terminate`. (RESPLE callbacks already wrapped, Phase 1.5 D.) | Wrapped every Mapping sensor callback + `getEstCallback` in `try { … } catch (const std::exception&) { WARN_THROTTLE }` (drop the scan, keep running). `getEstCallback` also drops an est_window with non-positive `dt`. |
| 45 | **Livox count/size mismatch** in BOTH nodes: the 3 `livoxLidarCallback`/`*AVIA*` variants looped `points[i]` up to `point_num` and `reserve(point_num)` — a torn message with `point_num > points.size()` → OOB read (uncatchable UB) / `reserve` throw | Clamp `plsize = min(point_num, points.size())` before the loop/reserve in all 6 Livox callbacks. |

Not a gap (verified): every PointCloud2 sensor (Ouster / Hesai / mid360 /
generic, both nodes) derives its loop bound from the **deserialized vector size**
(`fromROSMsg`/`ingestPointCloud2`), so there is no count-field-vs-size mismatch
to exploit — their only exposure was the missing try/catch (now closed in
Mapping; RESPLE already had it + `isfinite` filtering). Validated TSan- and
ASan-clean on the HelmDyn01 LIO replay.

#### 6.1 RESULTS — rmw_zenoh re-validation, 2026-06-10 (deployment RMW)

The production deployment uses **rmw_zenoh**, not the container/CI-default
Fast-DDS. Re-ran the TSan HelmDyn01 LIO replay with
`RMW=rmw_zenoh_cpp` (`scripts/sanitizer_replay.sh` now starts `rmw_zenohd` and
sets `RMW_IMPLEMENTATION`). Finding:

- **RESPLE code is race- and inversion-free under Zenoh too** — across both
  nodes, **0** `#0` racing frames in `libresple`, and **0** RESPLE lock-order
  inversions (all 96 inversions are in `librmw_zenoh_cpp`). Every racing access
  is a libsanitizer interceptor (`pthread_mutex_lock`/`pthread_create`/`memcpy`/
  `malloc`/`send`/`recv`) on a Zenoh runtime thread. Node healthy (LIO init,
  knot plateau 600).
- **But Zenoh's runtime is far noisier under TSan than Fast-DDS** — ~218 data
  races + 96 lock-order inversions, ALL inside `librmw_zenoh_cpp` / the
  uninstrumented zenoh-c async runtime. Not a RESPLE bug, not fixable from
  RESPLE, and too broad to suppress cleanly.

**Conclusion:** the meaningful gate (zero RESPLE-code races) passes under BOTH
RMWs. A literal zero-report TSan run is practical only on Fast-DDS (2 documented
suppressions); Zenoh's middleware noise would need a large brittle suppression
set. **Recommendation: gate TSan on Fast-DDS; treat Zenoh as validated for
RESPLE-code cleanliness as the deployment RMW.** ASan was not re-run under Zenoh
(the TSan racing-access analysis is the relevant evidence here).

### 6.2 Bag-replay CI smoke (last open Phase 5 row)

Goal: a CI job that replays a SHORT bag (30–60 s is enough) and fails on
regression — the missing end-to-end gate above the unit/concurrency tiers.

Design:
- New `scripts/run_bag_smoke.sh <bag_dir> <expected_pose_file>`:
  1. Build workspace (reuse `scripts/build_workspace.sh`).
  2. Launch `resple_pointcloud2.launch.py config_file:=<smoke config>`
     (or the sensor-specific launch matching the bag).
  3. `ros2 bag play` to completion + grace period; SIGINT the nodes.
  4. Assertions, all from a recorded `resple_diagnostics` + `odom` capture:
     - final pose within tolerance of `<expected_pose_file>` (golden pose
       from a verified run; position ‖Δp‖ < 0.5 m, yaw < 5° for a short
       indoor bag — tighten once measured variance is known);
     - `filter_state` never DIVERGED; `iekf_numerical_failures == 0`;
     - node exit codes 0 (shutdown hardening regression check).
  5. Subscriber tooling: a small C++ capture node compiled ad-hoc (the
     `rclpy` CLI may be unusable in minimal sandboxes — see the diagprobe
     pattern from the Phase 4 verification; consider committing it under
     `resple/test/tools/diag_capture/`).
- Bag hosting: CI runners can't assume a large artifact. Options, in order
  of preference: (a) trim a 30 s bag to < 100 MB and store with Git LFS;
  (b) a release-asset download step with checksum; (c) keep the job
  `workflow_dispatch`-only on a self-hosted runner with local data.
- CI wiring: separate job in `unit-tests.yml`, `if:` gated on the bag
  being retrievable so its absence skips rather than fails.

### 6.3 §3.2 plane-fit threshold tuning

#### Accuracy baseline — HelmDyn01 LIO, 2026-06-10 (default config)

Established before any tuning, as the reference to beat. RESPLE in LIO
(`if_lidar_only:false`), clean Release build, full HelmDyn01 replay; estimator
`/odom` captured to TUM (`scripts/record_tum.py`) and compared to the mocap GT
(`scripts/traj_eval.py`, cross-checked with `evo`).

- **APE (translation, SE3-aligned): RMSE 4.7 cm** (mean 4.2, median 3.8,
  max 29 cm) over a **199.9 m** trajectory = **0.021 %** of path length.
  evo agrees (rmse 4.74 cm). `/odom` is 100 Hz, monotonic.
- LIO accuracy is excellent — the previously-observed global-map "smearing"
  was an **LO-mode** artifact, not present in LIO.
- **GT caveat:** the HelmDyn mocap GT *orientation* is unreliable (flips:
  95th-pct consecutive step ~120°, max 180°), so RPE / rotation metrics against
  it are meaningless — it is a **position-only** reference (matches the dataset
  readme, which provides only a translation offset `t_L_gt`). Use ATE/APE
  translation for HelmDyn accuracy.
- **Tooling note:** replay scripts must launch the node *executables directly*,
  not via `ros2 run` — killing the `ros2 run` wrapper leaves the node binary
  alive, and leaked `/odom` publishers corrupt later runs (observed: a stray
  identity-pose publisher polluted a first attempt). `scripts/*.sh` now direct-
  exec + `pkill -x` on teardown.

#### Known characteristic — real-time map deskew uses trailing-edge spline knots (aggressive-motion azimuth smear), 2026-06-10

**Observation.** On HelmDyn01 the accumulated map (`/global_map`, and equally
`/current_scan`) smears in **azimuth/yaw** under the dataset's violent
helmet motion — in BOTH LO and LIO. The *trajectory* (converged poses) looks
good and APE is 4.7 cm; it's the *map* that smears, and the live `odom→base`
TF visibly jitters in yaw.

**Ruled out (in order):**
- *Timing / deskew offset* — the Mid360 IMU+LiDAR are hardware-synced;
  `header.stamp == timebase`, `offset_time` clean 0–50 ms, and both the RESPLE
  and Mapping paths compute `time_ns = frame_start + ms2ns(intensity)`
  identically. `lidar_time_offset = 0` is correct; no static sweep helped and
  no dynamic td estimation is warranted for a synced unit.
- *Point budget* — `num_points_upd` 100→300 made no visible difference.
- *IMU extrinsic `q_lb`* — `config_tudorun01.yaml` (same Mid360 rig, same
  identity `q_lb`/`t_lb`) ships as **LIO and renders crisp**, and the IMU is
  fused directly in the body frame (`Estimator.h:448`); `q_lb` only rotates
  LiDAR *points* (`Estimator.h:485`), so a wrong `q_lb` would *tilt* the cloud,
  not *smear* yaw.
- *Our pipeline* — **TudoRun01 (same sensor, same config style, smoother
  runner motion) renders crisp** in this exact container/pipeline. So the smear
  is HelmDyn-specific, not systemic.

**Root cause.** RESPLE is a recursive sliding-window estimator. The map
(`mapIncremental`) and `/current_scan` are deskewed **at processing time using
the spline's freshest, trailing-edge knot(s)** — which are the *least
converged* (only the current scan + IMU constrain them; future scans haven't
refined them yet). Under HelmDyn's fast yaw those edge knots jitter, so each
scan is placed with a slightly-off yaw and the accumulation smears. The
*trajectory* you see is the refined/converged estimate, so it stays smooth.
This is the inherent **real-time-output vs. smoothed-estimate gap**; only
extreme motion exposes it. TudoRun's gentler motion keeps the edge knots good.

**Mitigations and their trade-offs:**
- **Map lag (IMPLEMENTED 2026-06-10/11, `map_deskew_lag_knots`, default 8).**
  Build/deskew the map from poses *behind* the edge — i.e., after later scans
  have refined them — trading a small output latency for crispness.
  Implementation: the Mapping worker holds each scan until
  `spline_max - t_end >= lag × dt` (`MappingBase::processScan` gate); the held
  scan is then deskewed with knots that `updateKnots`/`setOneStateKnot` have
  already overwritten with refined est_window values. The publishPath tip is
  lagged identically, and pubOdom's odom→base lookup is **time-paired to the
  tip stamp** (previously Time(0)/latest — that pairing put full body motion
  over a 50–180 ms gap into the map→odom TF). `0` restores bleeding-edge.

  **Convergence horizon (why 8 and not more):** the IEKF updates only the
  last 4 RCPs and `getSplineMsg` resends only the last 5 knots, so a knot is
  final — in the estimator AND at the Mapping replica — once ~4 behind the
  edge. Cubic interpolation at scan time `t` reads knots up to `idx(t)+2`,
  so every knot a scan touches is final once the edge is ≥6 knots past it;
  8 adds margin. Beyond that, lag buys latency, not accuracy: at `knot_hz`
  100 the map runs 80 ms behind, far inside the accepted 0.5 s map-latency
  budget (requirement 2026-06-11: odometry must stay real-time; the map and
  `map→odom` may lag up to ~500 ms if that buys accuracy). A larger budget
  would only matter if the est_window protocol were widened AND the
  estimator re-optimized older knots — it does not (recursive-by-design).
  Pending bag validation: re-render HelmDyn01 + R_Campus and confirm the
  azimuth smear collapses; sweep lag ∈ {0, 4, 8} to confirm the horizon.

- **Internal-map insertion lag (CAPABILITY LANDED 2026-06-11,
  `map_insert_lag_knots`, default 0 = off).** The display-map lag above does
  not touch the deeper instance of the same disease: `processData`
  world-fixes points immediately after the IEKF (`pointBodyToWorld` with the
  freshest knots — upstream `main` does the identical thing) and the async
  `mapIncremental` inserts them into the ikd-Tree. Trailing-edge jitter is
  thereby baked into the REFERENCE map the IEKF matches against, and the
  error feeds back into every subsequent estimate ("registration errors
  introduced at an earlier stage remain in the map and affect all subsequent
  estimates" — retrospective map refinement, arXiv:2503.21293; SLICT admits
  scans to its map only at marginalization time for the same reason,
  arXiv:2211.03900; the RESPLE paper itself frames map maintenance as
  happening when "active RCPs transition into idle state", arXiv:2504.11580,
  which is closer to lagged insertion than to the shipped insert-at-edge).
  Implementation: with lag > 0 the worker stages body-frame points
  (`map_insert_staging_`, worker-thread-only) and releases them once the
  edge is `lag` knots past their stamps, deskewing with final knot values
  under `spline_mutex_`. `/current_scan` publishes the same released points,
  so it inherits the lag. Trade-off: the reference map is missing the last
  `lag × dt` of points (~1 scan at 10 Hz / 80 ms) — marginal near-field
  sparsity against a permanently crisper reference. **Off by default
  (decision-gate rule: this alters the odometry feedback loop); recommended
  trial value 8.** Bag experiment: HelmDyn01 + R_Campus APE with
  `map_insert_lag_knots` ∈ {0, 8}, after the display-lag sweep isolates the
  display-side effect.

  **Related work map (2026-06-11 survey)** — where each thread of the smear
  problem sits in the literature, for designing follow-ups after the bag
  experiments:
  - *Lagged/converged map admission (what we shipped):* retrospective map
    refinement (arXiv:2503.21293) — lag queue, promote to map after pose
    convergence; SLICT (arXiv:2211.03900) — scans enter the map only at
    sliding-window marginalization. Both validate the
    `map_deskew_lag_knots` / `map_insert_lag_knots` design.
  - *Uncertainty-weighted maps (the alternative to binary lag):* VoxelMap
    (arXiv:2109.07082) propagates BOTH LiDAR noise and POSE-ESTIMATE
    covariance into per-plane uncertainty, then weights matching by it —
    instead of delaying insertion, insert immediately but downweight
    edge-pose points until "empirical convergence of plane uncertainty".
    IMPLEMENTED in cheap form (2026-06-11): `map_insert_cov_gate_deg` —
    staged points release early when the edge-pose orientation std (from
    `getLastPoseCovariance`, the proper spline-Jacobian propagation) drops
    below the gate; the fixed-knot hold remains the aggressive-motion
    ceiling. Full VoxelMap-style per-plane uncertainty remains future work.
  - *Estimate-side fixes for aggressive motion (the part lag cannot fix):*
    Point-LIO (10.1002/aisy.202200459) updates per-point at 4–8 kHz, removes
    in-frame distortion by construction, and survives IMU saturation at
    75 rad/s — the benchmark for HelmDyn-class motion. Coco-LIC
    (arXiv:2309.09808) places B-spline knots NON-UNIFORMLY by motion
    intensity; ATI-CTLO (arXiv:2407.20619) adapts the temporal interval
    likewise; FR-LIO (arXiv:2302.04031) splits scans into sub-frames by
    motion intensity and smooths within an iterated Kalman smoother window.
    Shared lesson: a fixed `knot_hz` under-fits violent yaw — adaptive knot
    density is the principled estimate-side remedy if the lag sweep + parity
    A/B leave residual smear on HelmDyn01. The DETECTION side is implemented
    (2026-06-11): `knot_rotation_warn_rad` checks each knot's final
    `ort_delta` norm (= rotation absorbed per knot interval) as it leaves the
    est window, WARNs + counts in `/diagnostics` when the motion exceeds what
    `knot_hz` resolves — so bag runs now measure under-resolution directly
    instead of inferring it from smear renders.
  - *Spline theory + lineage:* the ETH continuous-time estimation survey
    (arXiv:2411.03951) is the canonical reference for the knot-count/order
    vs accuracy/cost trade-off; Cioffi et al. (RA-L 2021, CT-vs-DT SLAM)
    formalize when continuous-time wins. SFUISE (arXiv:2301.09033) is this
    group's own predecessor (recursive sliding-window spline fusion) —
    consult it before touching the RCP recursion.
  - *Degeneracy detection (feeds the open §3.2 `plane_min_cond_ratio`
    decision):* the literature gates the OPTIMIZATION, not the per-plane
    fit — X-ICP projects normalized Jacobians onto the Hessian eigenspace
    for per-axis NONE/PARTIAL/FULL localizability; LION/others threshold
    the Hessian condition number; GenZ-ICP (arXiv:2411.06766) adaptively
    reweights instead of gating; "Informed, Constrained, Aligned"
    (arXiv:2408.11809) is a field comparison of these methods. Our QR
    pivot-ratio gate is per-correspondence — a cheap per-update diagnostic
    on the stacked LiDAR Jacobian's eigenspectrum (the H rows already
    exist in pt_meas) would match the literature-standard signal and could
    decide §3.2 with better evidence than the plane-fit-level gate alone.
    DETECTION IMPLEMENTED (2026-06-11), report-only: per update, the
    per-point-normalized constraint information matrices E_tt = Σnnᵀ/N and
    E_rr = Σ(p×n)(p×n)ᵀ/N as SEPARATE 3×3 blocks (a joint 6×6 mixes
    meter-scaled rotation rows with unit normals → scale-dependent
    condition number, the field-analysis pitfall), min-eig + condition for
    each in `/diagnostics` ("Localizability ..."). Hard gating deliberately
    NOT implemented: arXiv:2408.11809's own conclusion is that eigenvalue
    thresholds are brittle across environments — collect bag distributions
    first, then decide §3.2 remediation (GenZ-ICP-style reweighting being
    the literature favorite over hard gates).
  - *Map data structure:* Faster-LIO's iVox (parallel sparse incremental
    voxels) trades slower per-query k-NN (~2.76 vs ~1.42 µs/point) for
    O(1) insertion and NO REBUILD THREAD. Note well: the ikd-Tree rebuild
    thread is the root of hazards 33–35 (Phases 2.4/2.5, the costliest
    concurrency work in this package) — an iVox-class structure would
    delete that hazard class outright, which is a stronger motive here
    than raw speed. Surfel-LIO (arXiv:2512.03397, Z-order voxel hashing +
    precomputed surfels) is the newer same-family option. DELIBERATELY NOT
    implemented as a drive-by (2026-06-11): Faster-LIO's own numbers show
    per-query k-NN ~2× SLOWER than ikd-Tree (≈2.76 vs ≈1.42 µs/point — its
    wins are insertion + parallelism), and RESPLE's hot loop is
    k-NN-dominated (num_match_points plane fits per point), so a naive swap
    risks a net regression. If attempted: own project, benchmarked on our
    bags, justified by the hazard-class deletion.
  - *Validation datasets beyond the current bags:* the Hilti SLAM
    Challenge / Hilti-Oxford datasets (arXiv:2208.09825; 2021–2023
    editions, handheld + robot-mounted, deliberate shaking/swinging,
    narrow stairs, dark corners) provide MILLIMETER-accurate control-point
    ground truth — directly useful because HelmDyn's mocap orientation GT
    is unreliable (§6.3 baseline note), so Hilti sequences can quantify
    orientation accuracy under aggressive motion where HelmDyn cannot.

  *Main-vs-lyrical logic audit (2026-06-11):* before trusting the
  "inherent real-time gap" framing, the map path was diffed against upstream
  `main`. The Mapping node is logic-equivalent (same `t_end <= maxTimeNs`
  bleeding-edge gate, same per-scan — not accumulated — `/global_map`
  publication, same per-point deskew; the staging swap and 200-scan cap only
  add ≤50 ms latency / backlog bounds). If `main` renders the same bag
  crisper, the delta is in the **estimator's numeric path**, where several
  default-on changes accumulated: `EIGEN_USE_BLAS` (different FP
  rounding/order — the CMake note records it initially destabilized the
  covariance update), threaded k-NN/association (`num_threads: 5`,
  FP-order changes), async background `mapIncremental`, and
  `EIGEN_INITIALIZE_MATRICES_BY_NAN` no longer set in Release. Each is a
  small perturbation; the trailing-edge knots are exactly where small
  perturbations are least damped. Decisive experiment (bag-gated): same bag
  through an `origin/main` build vs a parity-configured current build
  (`doc/PARAMETERS.md` § "Reproducing the original (`main`) behavior"),
  compare `evo_ape` + map renders.

  *Operator report (2026-06-11):* smearing was observed on the **hardened
  build** on HelmDyn01 AND **R_Campus** (LIO, Livox Avia, handheld-grade
  motion), among other datasets, while the operator's recollection is that
  upstream `main` rendered these crisp. Two implications: (1) the smear is
  NOT confined to HelmDyn-grade violent motion — ordinary walking/handheld
  yaw oscillation suffices, so the original "only extreme motion exposes
  it" framing above is too narrow and the map-lag default matters in
  normal operation; (2) the regression hypothesis (estimator numeric-path
  deltas, previous paragraph) is strengthened and the main-parity A/B is
  promoted to the highest-priority bag-gated experiment. Suggested order:
  first re-render R_Campus on the current build with `map_deskew_lag_knots`
  ∈ {0, 4, 8} (cheap, isolates the edge-deskew mechanism), then the
  main-vs-parity A/B (isolates the numeric regression).
- **Faster knot convergence — POTENTIAL ISSUES (document before attempting).**
  Trying to make the trailing knots converge sooner (more IEKF iterations,
  larger point budget, or tighter orientation process/measurement noise so the
  edge is pinned harder) is tempting but risky and fundamentally limited:
  - **Fundamentally bounded.** The newest knot is *under-observed by
    construction* — the measurements that would refine it (subsequent scans)
    have not arrived. No amount of edge-tuning manufactures information that
    isn't there yet; it can only reduce, not remove, the gap.
  - **Stability risk.** Over-trusting an under-constrained edge knot (tighter
    measurement weight / lower process noise) makes the filter fit noise on the
    freshest, weakest-conditioned part of the window → orientation overshoot,
    oscillation, and in the limit NIS divergence (the very failure mode the
    Phase 3.3 detector guards). Tighter orientation process noise also fights
    the IMU prior and reduces responsiveness to genuine fast motion — which can
    *worsen* fast-yaw smear.
  - **Compute / real-time cost.** More iterations or points per cycle raises
    per-frame cost; the worker already backlogs on denser feeds (observed
    `pc_buff` climbing on TudoRun at ~28 Hz). Falling behind real time makes
    the trailing-edge problem *worse*, not better.
  - **Net:** faster convergence is a tuning lever with diminishing returns and
    real downside; map lag addresses the actual mechanism (use refined poses)
    without destabilising the filter.
- **Accept (superseded 2026-06-10).** The original decision — position is
  accurate (4.7 cm) and the deployment sensor is an **Ouster on a rover**
  (motion much closer to TudoRun than to a head-worn helmet), so the
  edge-pose smear was unlikely to manifest in production. Superseded by the
  map-lag implementation above: the fix is cheap (latency-only, off-switch via
  `map_deskew_lag_knots: 0`), so it now ships enabled instead of relying on
  the deployment motion staying benign.

Goal: decide the production `plane_min_cond_ratio` (degeneracy guard,
currently 0 = off) and sanity-check `nn_max_sq_dist` / `plane_fit_thresh`
per sensor.

Procedure (per candidate bag, ideally one normal + one degenerate):
1. Baseline run with defaults; record `resple_diagnostics` and the
   trajectory (`odom`).
2. Sweep `plane_min_cond_ratio` over {1e-4, 1e-3, 1e-2, 5e-2} (QR pivot
   ratio is scale-invariant; expect the interesting region between 1e-3
   and 1e-2). One run each.
3. Compare per run:
   - trajectory error vs reference (e.g. `evo_ape` against ground truth,
     or against the baseline when no GT exists);
   - funnel deltas: `passed_plane/passed_knn` (how many patches the guard
     rejects) and `used` (how much measurement support the IEKF retains —
     a guard that costs >10–20 % of `used` on NORMAL geometry is too hot);
   - NIS window mean (consistency should improve or hold on degenerate
     stretches if the guard is doing its job).
4. Decision gate: enable (set a default in the example configs +
   `readParameters`) only if degenerate-stretch error improves while
   normal-geometry `used` and trajectory error are within noise.
   Otherwise leave 0 and record the numbers here.

### 6.4 §3.3 recovery-policy validation (hold vs reset)

Goal: pick the production `nis_recovery_mode` default (currently "off").

Needs a bag with a REAL divergence episode. If none exists, manufacture
one deterministically from a good bag: e.g. corrupt the extrinsics
(`q_lb` rotated by ~10°) for one run, or pre-filter the bag to drop all
scans for a 2–3 s window mid-run (forces an IMU-only stretch and a
correspondence shock on re-entry).

Procedure: identical replay under `off` / `hold` / `reset`; compare:
- detection latency (first DIVERGED verdict timestamp — same for all);
- during the episode: `off` publishes the bad pose (downstream EKF would
  ingest it), `hold` gates output (measure the outage duration via
  `recovery_hold_active`), `reset` reinflates covariance
  (`recovery_resets` count, watch for reset thrash — repeated resets in
  one episode mean the threshold or the reinflation is wrong);
- after the episode: time until NIS mean returns < `nis_warn_ratio`, and
  end-of-run trajectory error per mode.
Decision gate: `hold` is the expected production winner for the
odom-EKF-fed deployment (a gap is recoverable downstream; a poisoned pose
is not). `reset` wins only if it recovers materially faster without
thrash. Record the choice in the workspace config and the params docs.

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
