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
| 0   | Instrument before changing code | **Complete (code); user bag-replay pending** | `74f9078` |
| 1   | Safety fixes (initial) | **Complete** | `512da1d` |
| 1.5 | Defensive crash-hardening (3-pass) | **Complete** | `14e9be8` |
| 1.6 | Bug-chase session 2026-05-01: `__libc_free` SIGSEGV in `KD_TREE::Add_Points` | **Complete (this commit)** — see "Phase 1.6" below | this commit |
| 2   | Concurrency hardening | **2.1/2.2 done (1.5); 2.4 Push_Down race + 2.5 #2 deadlock + 2.6 two data-path races FIXED + TSan-verified; 2.3 pending data; 2.5 #1 rebuild-vs-mutator race deferred** — see below | this commit |
| 3   | Spline / mapping accuracy | **3.1 knot pruning IMPLEMENTED (this commit)** — gate satisfied by the 2.6 live measurement; 3.3 detection done (recovery policy pending); 3.2 / 3.4 pending | this commit |
| 4   | Diagnostics publisher | Can start after Phase 3 begins | — |
| 5   | Regression tests | Last | — |

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

### What's still open

- **Verify the Eigen-macro fix** by running sim post-commit. If gone, close
  this entry. If still firing, switch to `EIGEN_MALLOC_ALREADY_ALIGNED=1`.
- **Phase 2.3** (bounded input buffers) — Pass 0 diagnostic data still
  needed before tuning.
- **Phase 3.x** — unchanged; gated on Phase 0 trajectory benchmarks.
- **Push_Down race** (now documented) — fix would require a substantial
  refactor of upstream HKU-MARS code; deferred.
- **Operational safety net** — Mapping receives `est_window` from RESPLE.
  When RESPLE crashes and respawns, Mapping holds a stale spline window
  and queries it with timestamps far outside the new spline's range. The
  Phase 1.5 B clamp catches this safely, but it floods the log. Worth
  adding a "spline-discontinuity" reset in Mapping's `getEstCallback`
  when start_t jumps backward.

---

## Phase 2 — Concurrency hardening

**Status: 2.1 fixed (Phase 1.5 K1); 2.2 partially fixed (Phase 1.5 Fix C);
2.3 still pending Phase 0 data; 2.4 (Push_Down race) IMPLEMENTED + TSan-verified;
2.5 #2 (rebuild lock-order deadlock) FIXED + TSan-verified; 2.5 #1
(rebuild-vs-mutator data race) deferred.**

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

### 2.3 Bounded input buffers

`pc_buff` (per-LiDAR) and `imu_int_buff` are unbounded `std::deque`s. If the
worker stalls, memory grows without backpressure. Fix:
- Parameter-configurable max size (default: sized from Phase 0 measured peak
  × 2).
- Drop-oldest on overflow.
- Counter published in diagnostics (`… dropped this window`).

**Decision gate:** only do this if Phase 0 shows the buffers trend up under
normal operation. On a clean system they should empty every IEKF cycle.

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

### 2.5 Rebuild-path hazards surfaced by the 2.4 stress test — **#1 DEFERRED, #2 FIXED**

**Status: found while TSan-verifying 2.4; both pre-existing (present with AND
without the 2.4 change) and inherited from upstream HKU-MARS. #1 (rebuild-vs-
mutator data race) remains deferred and suppressed in
`resple/test/tsan_suppressions.txt`. #2 (the lock-order inversion / deadlock)
is FIXED this commit.**

1. **(DEFERRED) Rebuild-thread vs. mutator data race on `KD_TREE_NODE` fields.**
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

**Remaining work on #1 (deferred):** a complete fix needs consistent lock
ordering between the mutators and the rebuild thread — e.g. a recursive
`working_flag_mutex` so a mutator can hold it across its whole op (excluding the
rebuild thread's critical sections) without self-deadlocking on the internal
boundary acquisition; or routing all rebuilds through the single background
thread; or making the few raced `KD_TREE_NODE` pointer/aggregate fields atomic.
Each needs bag-replay validation (no bag available yet) before landing.

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
(and revisit `nn_max_sq_dist` per sensor) by benchmarking trajectory error on
a known-good bag, watching the new funnel counters.

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

**Remaining (recovery policy):**
- Action on trip: either (a) hold last estimate + skip publish until a stable
  scan recovers it, or (b) reset filter with the last good pose. This is a
  policy decision — (a) is safer operationally, (b) recovers faster from a
  real divergence. Detection currently logs + escalates diagnostics to ERROR
  so downstream consumers can gate on `/diagnostics`; no automatic reset yet.
- Optional: covariance trace / `λ_min` metrics on a dedicated status topic
  (the NIS verdict largely subsumes them for divergence purposes).

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

**Status: can start after Phase 3 begins.**

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
