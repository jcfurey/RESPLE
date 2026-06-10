# CLAUDE.md — RESPLE

Guidance for Claude Code when working inside `src/packages/localization/resple/`.
This package is vendored from the upstream ASIG-X RESPLE repo with workspace-local
modifications (lifecycle node, diagnostics, action server, lock hardening). The
upstream README covers usage and datasets; this file covers the architecture and
hazards you need to understand before editing.

## What it is

RESPLE is a recursive B-spline state estimator for 6-DoF odometry from one or
more LiDARs, optionally fused with an IMU. Four modes are supported upstream
(LO / LIO / MLO / MLIO). On the Rover MAX 150 it would run **LIO** —
`if_lidar_only: false` in `src/settings/params/localization/resple.yaml`,
with the Ouster IMU fused into the IEKF continuously (not just gravity init).

**Current production state (2026-05-01): RESPLE is OFF.**
`env.d/20-features.env` has `run_resple=False`. The active LiDAR-inertial
odometry source is DLIO, feeding the `robot_localization` odom EKF. RESPLE
remains fully wired for one-line revert (`run_resple=True` in
`20-features.env`). See `rovermax_ws/CLAUDE.md` "Localization Architecture"
for the active stack diagram.

Integration point **when enabled** (one-line revert):

```text
Ouster OS1-16 points (10 Hz) ─┐
Ouster IMU (100 Hz) ──────────┴─→ RESPLE (B-spline LIO) ─→ /localization/resple/odometry (10 Hz)
                                                                       │
                                                                       ▼
                                                                   odom EKF (odom1)
                                                                       │
                                                                       ▼
                                                       TF: odom → base_footprint
```

The odom EKF owns the `odom→base_footprint` TF. RESPLE publishes its pose as a
sensor source with `odom/publish_tf: false`; see
`src/settings/params/localization/odom_localization.yaml` for the consumer
side (`odom1` input).

Bug-chasing this package today therefore means either: (a) replaying recorded
production bags under sanitizer builds, or (b) flipping `run_resple=True`
(and `run_dlio=False` to avoid the TF-owner conflict the master launch
hard-fails on) to reproduce live. Pick (a) first — it's deterministic.

## Directory layout

```text
resple/
├── resple/              # core package (node + library)
│   ├── CMakeLists.txt   # build flags — see Build section below
│   ├── src/
│   │   ├── RESPLE.cpp   # ~90 KB — lifecycle node, callbacks, processData loop, locks
│   │   ├── Mapping.cpp  # ~50 KB — ikd-Tree map updates, publishing, save-map action
│   │   └── gpu/         # optional CUDA k-NN path (RESPLE_USE_CUDA, off by default)
│   ├── include/
│   │   ├── Estimator.h  # IEKF (propRCP, updateIEKFLiDAR, updateIEKFLiDARInertial), Joseph-form covariance
│   │   ├── SplineState.h # cumulative B-spline over SO(3) × R^3 — knot container + interpolation
│   │   ├── Association.h # point-to-plane correspondence + deskew via spline query
│   │   ├── ikd-Tree/    # vendored HKU-MARS incremental kd-tree
│   │   ├── gpu/cuda_knn.h
│   │   └── utils/       # math_tools.h, common_utils.h, eigen_utils.hpp
│   ├── config/          # upstream per-dataset YAML (reference only)
│   └── launch/          # upstream per-dataset launch (reference only)
├── {AviaResple,HAP360,Mid70Avia}_msgs/   # custom LiDAR types — we use Ouster, not these
├── estimate_msgs/       # Knot / Spline / Estimate msg + SaveMap action
└── README.md            # upstream docs (datasets, citation)
```

Workspace config lives in **`src/settings/params/localization/resple.yaml`**, not
under this package. The upstream `config/` dir is kept for reference but not used
at runtime. Launch is driven from `settings_erdc` via `run_resple=True` (see
`env.d/20-features.env`).

## Execution model

### Node type

`rclcpp_lifecycle::LifecycleNode` with `use_intra_process_comms(true)`. State
transitions exposed via standard ROS 2 lifecycle: `configure → activate →
deactivate → cleanup → shutdown`. Subscribers and the worker thread are
**created in `on_activate` and torn down in `on_deactivate`** — not in the
constructor. If you add new state, hook both sides.

### Threads

| Thread | Source | What it does |
| --- | --- | --- |
| ROS executor (default) | rclcpp | Drives lifecycle transitions and the diagnostic updater |
| `sensor_cb_group` (MutuallyExclusive) | `RESPLE.cpp:113` | All IMU + LiDAR subscriptions; callbacks run serialized on this group. Each one just appends to a buffer under a mutex, never does heavy work. |
| `processing_thread_` | `RESPLE.cpp:205` | Main IEKF loop at 20 Hz (`rclcpp::Rate`). Drains lidar/IMU buffers, runs `propRCP` → `updateIEKF*` → deskew → map update kickoff. |
| `map_update_future_` | `RESPLE.cpp:474` (`std::async`) | Per-cycle background task: `mapIncremental` + `lasermapFovSegment` + `publishFrameWorld`. Worker waits for prior future before swapping buffers. |
| `save_map_thread_` | `RESPLE.cpp:672` | Per-invocation action-server thread for `SaveMap` action. Joined on deactivate. |
| OpenMP pool | `#pragma omp parallel for` | Inside the IEKF (`num_threads_` param, default 4) for point→world transform and k-NN search. |

**Global state warning:** `ikdtree` is a translation-unit-level global in
`RESPLE.cpp:54` (and `g_cuda_map` at line 56 under CUDA). This means the
package can only host one node instance per process. Do not try to spin two
RESPLE nodes in one composable container.

### Lifecycle state vs. callback safety

The IMU callback snapshots both `if_lidar_only` and `if_init_filter` under
`m_buff` in one transaction (`RESPLE.cpp` `getImuCallback`). This is the fix
for a prior crash where the callback could see `if_init_filter == true`
before the spline pointer was actually initialized. The ordering requirement
is:

1. Fully construct the `SplineState`.
2. Point `spline` at it.
3. Flip `if_init_filter = true` under `m_buff`.

**Do not reorder.** The state flags `if_init_filter`, `if_init_map`,
`if_lidar_only`, `localmap_initialized_` are now `std::atomic<bool>`
(Phase 1.5 Fix C). This closes the bare-bool data race the worker had on
`if_init_filter` / `imu_int_buff.empty()` reads. The `m_buff` snapshot
discipline above is still required for the IMU callback's own LO/LIO branch
selection — atomicity alone doesn't give a coherent multi-flag transition.

The full atomic-enum state machine refactor (Phase 2.2) is no longer urgent
but is still a code-clarity win.

### Callback-drain barrier on `on_deactivate`

Both `RESPLE` and `Mapping` `on_deactivate` sleep for **100 ms** after
resetting subscriptions, before returning. Reason: `Subscription::reset()`
drops the application's strong ref but the executor still holds one for any
callback already in flight on another thread (sensor_cb_group is
MutuallyExclusive, so at most one). Without this barrier, `on_cleanup`'s
state teardown (`lidars_data.clear()` etc.) can deallocate a `LidarData` /
TF buffer / publisher while a callback is mid-execution holding a reference
to it → SIGSEGV. Temporary fix until Jazzy exposes `wait_for_callbacks` on
`CallbackGroup`.

### Lifecycle re-entry safety

`declare_parameter` throws `ParameterAlreadyDeclaredException` if called
twice. All param reads in this package go through `CommonUtils::readParam`
(which guards with `has_parameter`) **except** three direct `declare_parameter`
calls in RESPLE.cpp's `on_configure` (`num_threads`, `num_match_points`) and
`on_activate` (`lidars`). These three are now guarded with `has_parameter`
checks (Phase 1.5 R1+R2) so a `deactivate → cleanup → configure → activate`
cycle does not crash.

When adding a new direct `declare_parameter` call, **either** wrap with a
`has_parameter` guard, **or** use `CommonUtils::readParam` instead. The
latter is preferred (matches everything else in `readParameters()`).

`on_activate` also refuses to overwrite a still-joinable `processing_thread_`
(would call `std::terminate` via `std::thread` move-onto-joinable). If the
ERROR fires, the previous deactivate didn't complete cleanly — investigate
that, don't paper over.

## Locking and lock ordering

There are four mutexes that matter. Get this wrong and you deadlock or corrupt
the map.

| Lock | Type | Protects | Held by |
| --- | --- | --- | --- |
| `lidar_data.mtx_pc` | `std::mutex` | Per-LiDAR input buffer (`pc_buff`, `t_buff`) | Sensor callback (brief), `processData` drain (brief) |
| `m_buff` | `std::mutex` | IMU buffer (`imu_int_buff`) + `if_init_filter` | IMU callback, `initialization()`, activate/deactivate |
| `mtx_map_` | `std::shared_mutex` | ikd-Tree contents (`Add_Points`, `Delete_Point_Boxes`, `Nearest_Search`, `flatten_safe`) | **shared**: IEKF findCorresp/updateIEKF. **unique**: async `mapIncremental` + `lasermapFovSegment`, `SaveMap` action. |
| `spline_mutex_` | `std::mutex` | `SplineState` mutations and reads (`propRCP`, `updateIEKF*`, `pointBodyToWorld`, `publishFrameWorld`) | Worker (IEKF + deskew), async future (publishFrameWorld) |

**Lock ordering (enforced by convention, commented at `RESPLE.cpp:615-617`):**

```
mtx_map_  ─── always acquired BEFORE ───  spline_mutex_
```

When both are needed, take `mtx_map_` first. The worker's IEKF block at
`RESPLE.cpp:384-391` holds `mtx_map_` (shared) and then `spline_mutex_` — this
is the canonical pattern. The async map-update future does NOT hold them
simultaneously: it takes `mtx_map_` (unique) for the kd-tree mutation, releases
it, then takes `spline_mutex_` alone for the publish. That split is
deliberate — a slow ROS publish must not block the next IEKF cycle on the map
lock.

**Never do:**

- Take `spline_mutex_` first then `mtx_map_` (cycle → deadlock).
- Do blocking I/O (ROS publish, file write) under `mtx_map_` unique.
- Hold `mtx_map_` or `spline_mutex_` across a `rate.sleep()` or any point
  where the worker might exit.

### Buffer-swap protocol (map update)

`processData` accumulates `pc_world` + `accum_nearest_points` frame-by-frame,
then every ~100 ms swaps them into `pc_world_bg_` / `accum_nearest_points_bg_`
and hands those to the async map update (`RESPLE.cpp:459-490`). The wait on
`map_update_future_` before the swap serializes back-to-back map updates —
the next cycle cannot race a still-running previous one.

### `getPositionLiDAR` is pure-read (no `propRCP` side-effect)

Phase 1.5 (Pass 3) removed a `propRCP` call from `getPositionLiDAR` that
was mutating `Estimator::cov_rcp` from inside the async map-update lambda,
which holds only `mtx_map_` (unique) — not `spline_mutex_`. The mutation
was race-free only because the IEKF needs `mtx_map_` shared and was
blocked, but the implicit lock-coupling was fragile. The function is now
pure-read; if you re-add a mutating call here, take `spline_mutex_` first
or refactor the caller.

**It is pure-read but it still READS the spline** (`itpPosition` /
`itpQuaternion`), so its single caller `lasermapFovSegment` must hold
`spline_mutex_` around the call. The "race-free because the IEKF holds
`mtx_map_` shared" reasoning above does NOT cover the spline *read*: the
worker grows the spline via `collectMeasurements → addOneStateKnot` under
`spline_mutex_` **alone** (no `mtx_map_`), so `mtx_map_` unique does not
exclude that write. The async task reading the spline under only `mtx_map_`
raced with it (TSan, data-path sweep). Fixed by taking `spline_mutex_` in
`lasermapFovSegment` (order `mtx_map_ → spline_mutex_` preserved).

### ikd-Tree `Nearest_Search` always takes the shared lock

Phase 1.5 (K1) removed the lock-free fast-path that checked
`Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node` racy with the
rebuild thread's subtree swap. `Nearest_Search` now unconditionally takes
`search_rw_mutex_` shared. Cost: one uncontended atomic-increment per call
(rebuild only takes unique briefly during the swap). Worth it — the racy
fast-path was a HIGH-severity UAF window in the IEKF k-NN parallel-for.

### ikd-Tree `Push_Down` per-node locking (Phase 2.4)

`Push_Down(P)` propagates deletion flags to `P`'s children. Concurrent
searchers (the IEKF runs `findCorresp` as an OpenMP parallel-for, so many
threads call `Nearest_Search` → `Push_Down` at once) used to race on the
child nodes' flag fields. The fix:
- `Push_Down` takes `P`'s own `push_down_mutex_lock` for the body (serializing
  concurrent `Push_Down(P)` and protecting `P`'s flag clears) **and** each
  child's `push_down_mutex_lock` around that child's field writes (serializing
  a `Push_Down(P)`-writes-`C` against a `Push_Down(C)`). Lock order is strictly
  parent→child along tree edges → acyclic → deadlock-free; a thread holds at
  most two node mutexes. The call sites are now bare `Push_Down(root)` — the
  old trylock dance moved inside.
- The mutators that SET `need_push_down_to_*` on a subtree root (`Add_by_range`,
  `Delete_by_range`, `run_operation`'s PUSH_DOWN replay) take that node's
  `push_down_mutex_lock` around the write, closing the set-vs-clear lost-update.
- The racy `KD_TREE_NODE` fields (`TreeSize`, `invalid_point_num`,
  `down_del_num`, and the six deletion/propagation bools) are `std::atomic`, so
  the lock-free reads in `Search` / `Search_by_range` / `Search_by_radius` /
  `Criterion_Check` / `Update` are defined behaviour.

This makes multi-threaded k-NN safe by contract; the old `num_threads=1`
mitigation is retired. Verified with `resple/test/test_ikdtree_concurrency.cpp`
under ThreadSanitizer (reader-vs-reader `Push_Down` races 16→0). The
`search_rw_mutex_`↔`working_flag_mutex` rebuild deadlock (hazard 34) was also
fixed (Phase 2.5 #2); the formerly-deferred upstream rebuild-vs-mutator data race
(hazard 35, Phase 2.5 #1) is now also fixed via a recursive whole-op
`working_flag_mutex`; `resple/test/tsan_suppressions.txt` is empty.

## Build configuration

Flags are defined in `src/packages/localization/resple/resple/CMakeLists.txt`.
The workspace does **not** extend them via `colcon_defaults.yaml` — this
package opts out of the workspace's `-Wshadow -Wunused` so upstream compiles
clean.

| Flag | State | Notes |
| --- | --- | --- |
| `-O3` | On in Release | — |
| `-march=native` | On (`ENABLE_NATIVE_ARCH`) | Binary is non-portable across hosts. OK for single-host deployment. |
| `-ffast-math` | **Off** (dropped 2026-04-21) | Previously on; removed because it reassociates the Joseph-form `(I-KH)P(I-KH)^T+KRK^T` update and silences NaN/Inf divergence checks. |
| `-Wall -Wextra -Wshadow` | On | `-Wpedantic` / `-Wunused` subflags not added (upstream noise). |
| `ENABLE_TSAN` | Off | Mutually exclusive with `ENABLE_ASAN`. Switches to `-O1 -g -fno-omit-frame-pointer -fsanitize=thread`. |
| `ENABLE_ASAN` | Off | ASan + UBSan combined. Switches to `-O1 -g -fno-omit-frame-pointer -fsanitize=address -fsanitize=undefined`. |
| `EIGEN_INITIALIZE_MATRICES_BY_NAN` | Debug only | Estimator members are explicitly zero-initialized via in-class initializers so Release builds don't rely on this flag. |
| `ENABLE_OPENMP` | On | Required for parallel findCorresp / pointBodyToWorld. |
| `ENABLE_CUDA` | **Off** | CUDA k-NN path exists (`src/gpu/cuda_knn.cu`, `RESPLE_USE_CUDA`) but is not built by default. |
| `BLAS` | Detected if present (`-DEIGEN_USE_BLAS`) | Destabilized the simpler `(I-KH)P` update historically; safe now that the Joseph-form posterior is used. |

The library target is built once and linked by both the `RESPLE` node
executable and the `Mapping` standalone executable. The component registration
is `PLUGIN "RESPLE" EXECUTABLE RESPLE_node` — we launch the executable, not
the composable node.

### Building (full colcon build, incl. in a web session)

One command from the repo root:

```bash
./scripts/build_workspace.sh          # build --packages-up-to resple
./scripts/build_workspace.sh --test   # + colcon test
```

It assembles a clean workspace and builds against the real ROS 2 / PCL stack.
The dependency-light estimator-core tests build/run without ROS at all via
`./scripts/run_unit_tests.sh` (Eigen + GTest; the ikd-Tree concurrency test
additionally needs PCL and is gated on it).

Non-obvious things the script and the SessionStart hook handle (learned the
hard way bringing the full build up on Claude Code on the web):

- **The Livox message packages are already in this repo**, just under other
  directory names: `AviaResple_msgs`=`livox_interfaces`,
  `Mid70Avia_msgs`=`livox_ros_driver`, `HAP360_msgs`=`livox_ros_driver2` (plus
  `estimate_msgs`). No external clones — the script symlinks them into the ws.
- **ROS apt over http.** Some network policies 503 the TLS `packages.ros.org`
  but allow plain http; apt still verifies the GPG signature, so the hook adds
  an `http://` source as a fallback.
- **NumPy + the right Python.** rosidl's message generator does
  `find_package(Python3 ... NumPy)`. The image has several Pythons and
  `/usr/local/bin/python3` may be a 3.11 with no working NumPy; the script picks
  an interpreter that can import NumPy (prefers the ROS `python3.12`) and passes
  it as `-DPython3_EXECUTABLE`. The hook installs `python3-numpy`/`python3-dev`.
- **BLAS must be linked by test targets.** `-DEIGEN_USE_BLAS` is added globally,
  so every gtest that touches Eigen matrix products links `${BLAS_LIBRARIES}`
  (hook installs `libblas-dev`/`liblapack-dev`).

## Hardening status

Full phased roadmap and rationale in **[`HARDENING.md`](HARDENING.md)** (same
directory). Summary:

| Phase | Title | Status |
| --- | --- | --- |
| 0   | Instrumentation + sanitizer builds | code complete (`74f9078`); bag replay pending |
| 1   | Safety fixes (initial)             | complete (`512da1d`) |
| 1.5 | Defensive crash-hardening          | complete (`14e9be8`) — 13 fixes across 3 passes |
| 2   | Concurrency hardening              | 2.1 + 2.2 subsumed by 1.5; 2.3 capability landed (default-off scan cap); 2.5 #1 fixed |
| 3   | Spline / mapping accuracy          | 3.1 knot pruning done; 3.2 instrumented/parameterized (tuning pending bags); 3.3 detection + recovery (off/hold/reset) done; 3.4 radius pruning done (off by default) |
| 4   | Diagnostics publisher              | done (`estimate_msgs/Diagnostics` on `resple_diagnostics`, ~20 Hz typed; see HARDENING §4) |
| 5   | Regression tests                   | done except bag-gated smoke (CI: ROS-free + ASan/UBSan + ikd-Tree TSan jobs; Eigen-pin + STATIC_ROOT_NODE leak fixed via the ASan gate) |

### Known hazards (compact view)

The original audit identified 10 issues; Phase 1.5 added several more. Quick
reference (status as of latest pass):

| # | Hazard | Status | Phase |
| --- | --- | --- | --- |
| 1  | ikd-Tree `Nearest_Search` lock contract unverified | **fixed** (always-shared-lock) | 1.5 K1 |
| 2  | Init state machine is a bool pair (fragile) | **race fixed** (atomic bools); enum refactor optional | 1.5 C / 2.2 |
| 3  | `map_update_future_.valid()` racy read | **fixed** | 1 |
| 4  | Unbounded knot growth | **fixed** (sliding-window prune in both nodes: `SplineState::pruneFrontKnots` slides the idle window so retained-range interpolation is bit-identical; absolute est_window indexing kept via `totalKnots()`; param `spline_prune_keep_knots`, default 600, 0 disables) | 3.1 |
| 5  | Unbounded input buffers | **capability landed** (`max_scan_buffer` per-LiDAR drop-oldest, default 0=off; `max_imu_staging` default 2000 replaces the old hardcoded cap; drop counters in diagnostics + the Phase 4 msg) | 2.3 |
| 6  | `assert()` in hot paths → silent UB under `-DNDEBUG` | **fixed** | 1 |
| 7  | No divergence detection | **fixed** (NIS detector + `nis_recovery_mode` off/hold/reset; hold gates odom/TF while DIVERGED, reset reinflates the IEKF covariance to the prior) | 0 → 3.3 |
| 8  | Plane-fit outlier rejection incomplete | **instrumented + parameterized** (CorrespConfig: `nn_max_sq_dist`/`plane_fit_thresh`/`plane_min_cond_ratio` params, degeneracy guard off by default pending bag benchmark; per-window funnel counters in diagnostics) | 3.2 |
| 9  | Deskew out-of-window extrapolation | **fixed (clamp + counter)** | 1 + 1.5 A/B |
| 10 | `-ffast-math` on | **fixed** | 1 |
| 11 | `pointBodyToWorld` OOB on out-of-range t_ns (logged but not clamped by Phase 1) | **fixed** | 1.5 A |
| 12 | `SplineState::itpPose` / `prepareInterpolation` indexed `t_knots[idx0+i]` without bounds | **fixed** (defensive clamp + n_active cap) | 1.5 B |
| 13 | Worker read `imu_int_buff.empty()` without `m_buff` (deque-internals data race) | **fixed** | 1.5 C |
| 14 | Sensor callbacks throw `std::out_of_range` from `lidars.at()` → `std::terminate` | **fixed** (try/catch in 7 callbacks) | 1.5 D |
| 15 | `lidars_data.clear()` in `on_cleanup` while a callback is mid-execution | **fixed** (100ms drain barrier) | 1.5 E + M3 |
| 16 | `on_activate` re-launching a still-joinable `processing_thread_` → `std::terminate` | **fixed** (joinable guard) | 1.5 F |
| 17 | `declare_parameter` re-call on lifecycle re-cycle → SIGABRT | **fixed** (has_parameter guards) | 1.5 R1+R2 |
| 18 | Resource leak (`save_map_action_server_`, `tf_buffer_`, `tf_listener_`) on re-cycle | **fixed** | 1.5 R3 |
| 19 | Mapping `if_init_succeed` bare bool race + `spline_active_.init()` race | **fixed** (atomic + m_spline) | 1.5 M1+M2 |
| 20 | `getPositionLiDAR` mutated `cov_rcp` outside `spline_mutex_` (implicit lock-coupling) | **fixed** | 1.5 (Pass 3) |
| 21 | `lock_mappings()` non-RAII → throw between → deadlock | **fixed** (`ScopedMappingsLock`) | 1.5 M4 |
| 22 | ikd-Tree `Add_Points`/`Add_Point_Boxes`/`Delete_Points`/`Delete_Point_Boxes`/`Box_Search`/`Radius_Search` had the same K1 race as `Nearest_Search` (rebuild thread can free old subtree between lock-free check and operation) | **fixed** (per-branch shared lock around fast-path; slow path still uses working_flag_mutex to avoid lock-order inversion) | 1.5 K1 extension |
| 23 | Async map-update lambda had no try/catch → throw stuck `map_update_pending_=true`, exception silently dropped by future destructor → silent map staleness → eventual k-NN against stale tree → SIGSEGV downstream | **fixed** (try/catch + always clear pending) | post-1.5 |
| 24 | `Mapping::main()` LidarConfig ctor + buffs construction unguarded; `q_lb_v.at(3)` throw escapes `main()` → `std::terminate` → SIGABRT with no log | **fixed** (try/catch + RCLCPP_FATAL + clean exit) | post-1.5 |
| 25 | `lasermapFovSegment` `pos_lidar_max` initialized with `numeric_limits<double>::min()` (smallest *positive*, not most negative) → silent local-map drift for any pose with negative components | **fixed** (`::lowest()`) | post-1.5 |
| 26 | `SplineState::if_first` set true in `init()` and never cleared → `minTimeNs()` always returns `start_t_ns`, `start_t_ns - dt_ns` branch dead | **fixed** (flag + dead branch removed; comment notes intent if reintroduced) | post-1.5 |
| 27 | `getRCPs()` indexes `t_knots[num_knot - 4 + i]` — UB if `num_knot < 4`. Safe today (no pruning) but a Phase 3.1 trap | **fixed** (defensive zero return + invariant log) | post-1.5 |
| 28 | Livox callbacks (`livoxLidarCallback`, `livoxLidar2Callback`, `livoxAVIACallback`) read `livox_msg_in->points[0]` after gating only on `point_num` field — empty-vector OOB if publisher misbehaves | **fixed** (also check `points.empty()`) | post-1.5 |
| 29 | `RESPLE/Mapping main()` ignored `configure()`/`activate()` returns → executor spun on half-init node → confusing downstream SIGSEGV | **fixed** (return-value check + RCLCPP_FATAL + non-zero exit) | post-1.5 |
| 30 | `map_update_future_.wait()` had no timeout → if lambda hangs, worker permanently locks (no IEKF, no publishes, node alive but silent) | **fixed** (`wait_for(5s)` + skip cycle on timeout + ERROR log) | post-1.5 |
| 31 | `processData` worker loop body unwrapped → throw anywhere kills worker thread silently → "alive but doing nothing" | **fixed** (try/catch around iteration body, log + 50ms sleep + continue) | post-1.5 |
| 32 | `mapIncremental` passed NaN/Inf points straight to ikd-Tree `Add_by_point`, where `calc_dist` returns NaN, `<` comparisons all false, recursion takes wrong branch, tree state corrupts silently | **fixed** (skip non-finite points; surface counter via WARN_THROTTLE) | post-1.5 |
| 33 | `ikd-Tree::Push_Down` writes to children's flag fields holding only the parent's per-node lock → races on overlapping subtree paths (dominant caller: findCorresp's OpenMP parallel `Nearest_Search`) | **fixed** (Phase 2.4: `Push_Down` takes the parent's *and* the child's `push_down_mutex_lock`; mutator flag-SET sites take the node's lock; the racy node fields are `std::atomic`; `num_threads=1` mitigation retired). TSan-verified reader-vs-reader races 16→0 via `test_ikdtree_concurrency` | 2.4 |
| 34 | `working_flag_mutex`↔`search_rw_mutex_` lock-order inversion (real deadlock): Phase 1.5 K1 wrapped the *mutating* fast-path calls in `Add_Points`/`Delete_Points`/etc. in `search_rw_mutex_` shared, but those take `working_flag_mutex` at a deeper rebuild boundary — opposite order to the rebuild thread (`working_flag`→`search_rw` unique). TSan-confirmed; observed to hang | **fixed** (Phase 2.5 #2: drop the shared lock from the 5 mutating fast paths — mutators self-coordinate via `working_flag_mutex`, the upstream mechanism; K1's lock kept on the *search* fns + the genuine `Search_by_range` read). Verified deadlock-free under TSan | 2.5 |
| 35 | ikd-Tree background rebuild thread vs. concurrent mutator data race on `KD_TREE_NODE` fields (`father_ptr` at `multi_thread_rebuild` ~255 vs `Update` ~1564; ancestor Update-walk during swap) | **fixed** (recursive whole-op `working_flag_mutex`: the four public mutators hold it across their entire operation, excluding the rebuild thread's father_ptr read + swap ancestor Update-walk; `Rebuild()`'s rebuild_ptr trylock keeps it deadlock-free). TSan: pre-fix ~200 races on the new `RebuildVsMutatorRace` stress → 0, suppressions file emptied | 2.5 |
| 36 | `processData` lidar-buffer drain checked `while (!lidar_data.t_buff.empty())` OUTSIDE `mtx_pc`, racing the sensor callback's locked `t_buff.push_back` on the deque internals | **fixed** (Phase 2.6: check emptiness under `mtx_pc` — `while(true){ lock; if(empty) break; … }`). TSan data-path sweep, real-race count 18→0 | 2.6 |
| 37 | async map-update task read the spline (`lasermapFovSegment → getPositionLiDAR → itpPosition/itpQuaternion`) holding only `mtx_map_`, while the worker grows the spline (`collectMeasurements → addOneStateKnot`) under `spline_mutex_` ALONE (no `mtx_map_`) → race | **fixed** (Phase 2.6: take `spline_mutex_` in `lasermapFovSegment`; order `mtx_map_ → spline_mutex_` preserved). TSan-verified | 2.6 |
| 38 | `joinProcessingThreadBounded` (both nodes) dispatched `join()` onto a `std::async` task and `detach()`ed from the lifecycle thread on timeout — two threads racing on the same `std::thread` object (UB when the worker exited at the deadline; TSan runtime CHECK-abort in `pthread_detach`), and the async future's dtor un-bounded the wait | **fixed** (worker lambda release-stores `processing_thread_exited_` as its last action; bounded join polls the flag, then join-or-detach single-threaded) | 2.7 |

"Measuring" means Phase 0 added a diagnostic metric; the fix is scheduled but
gated on observing the signal. Do not implement a fix in category 4 / 5 / 7
without the Phase 0 bag data — see `HARDENING.md` for decision gates.

## Parameters you will hit often

Canonical values in `src/settings/params/localization/resple.yaml`:

| Param | Meaning | Current |
| --- | --- | --- |
| `if_lidar_only` | LO (true) vs LIO (false) | `false` — we run LIO |
| `knot_hz` | B-spline knot rate | `100` |
| `n_iter` | IEKF iterations per frame | `1` |
| `num_points_upd` | max LiDAR points per IEKF step | `300` |
| `ds_scan_voxel`, `ds_lm_voxel` | voxel downsampling (m) | `0.2` |
| `num_match_points` | k for k-NN plane fit | `5` (OS1-16 is sparse) |
| `nn_thresh` | point-to-plane distance threshold (m) | `0.5` |
| `cube_len` | local map cube size (m) | `1000.0` |
| `num_threads` | OpenMP threads | `4` |
| `spline_prune_keep_knots` | knots retained by the Phase 3.1 sliding-window prune (both nodes; 0 disables, <100 clamps to 100) | `600` |
| `nn_max_sq_dist` | §3.2 k-th-neighbor squared-distance gate (m²); search radius = sqrt of this | `5.0` |
| `plane_fit_thresh` | §3.2 esti_plane residual threshold (m) | `0.1` |
| `plane_min_cond_ratio` | §3.2 plane-fit degeneracy guard (QR pivot ratio; 0 = off, pending bag benchmark) | `0.0` |
| `map_prune_radius` | §3.4 keep only map points within this distance (m) of the pose; floored at 2×det_range; 0 = off (cube-only) | `0.0` |
| `nis_recovery_mode` | §3.3 divergence recovery: `off` (detect-only) / `hold` (gate odom+TF while DIVERGED) / `reset` (reinflate IEKF covariance to prior) | `off` |
| `max_scan_buffer` | §2.3 per-LiDAR raw-scan cap (scans, drop-oldest; 0 = unbounded) | `0` |
| `max_imu_staging` | §2.3 IMU staging cap (samples, drop-oldest; was hardcoded 2000) | `2000` |

The `cube_len` hardcoded default in code (`RESPLE.cpp:580`, value 2000) is
overridden by the param; check the logged value on startup rather than reading
the source.

## Working with this package

- **Don't modify the upstream files under `config/` or `launch/`** (dataset
  variants) — our runtime uses `settings_erdc` + our YAML. Upstream files are
  kept for reference and occasional benchmarking.
- **Do modify the workspace config** at `src/settings/params/localization/resple.yaml`.
- **Before adding a new per-sensor callback, check whether the generic
  `PointCloud2` lidar type already covers the sensor.** `lidar_type:
  PointCloud2` resolves x/y/z + time/intensity fields by name at runtime
  (`utils/point_cloud_adapter.h`, unit-tested) and handles relative or
  absolute-epoch per-point time in any datatype/endianness. Per-lidar YAML
  overrides: `time_field`, `time_unit` (auto|s|ms|us|ns), `intensity_field`.
  Template config: `config/config_pointcloud2.yaml`. Both RESPLE
  (`genericLidarCallback`) and Mapping (`GenericPC2Buff`) support it. A
  hand-written sensor struct is only needed for non-PointCloud2 transports
  (e.g. Livox CustomMsg).
- When adding a new sensor type, wire the callback through `sensor_cb_group`
  (not the default group) and push to a per-sensor `LidarData` struct with its
  own `mtx_pc` — do not add heavy work inline in the callback. **Wrap the
  callback body in `try { ... } catch (const std::exception& e) {
  RCLCPP_WARN_THROTTLE(...); }`** — every existing callback follows this
  pattern (Phase 1.5 D). `lidars.at(name)` throws `std::out_of_range` for an
  unexpected frame_id; without the catch the executor turns that into
  `std::terminate` → SIGABRT.
- When adding shared state read by `processData` and any callback, decide
  which existing mutex covers it. Do not introduce a fifth mutex without
  updating the lock-ordering rule above.
- When adding new bool flags shared between worker and callback threads,
  use `std::atomic<bool>` (matches the Phase 1.5 C pattern). Operator T()
  and operator= preserve existing call sites; cost is one atomic per
  read/write.
- When adding a `declare_parameter` call, prefer `CommonUtils::readParam`
  (which has a `has_parameter` guard built in) over the direct API. Direct
  `declare_parameter` calls must be guarded with `if (!has_parameter(...))`
  to survive lifecycle re-cycles.
- When adding a new spline reader (anything that calls `spline->itp*` or
  reads knot data), take `spline_mutex_` first. Knot DEQUE indices are local:
  with Phase 3.1 pruning active, absolute (protocol) index = local index +
  `numKnotsPruned()`; times never need translation (`start_t_ns` advances with
  the prune). Anything holding knot indices across cycles must use
  `totalKnots()` space (see the est_window publish gate in both nodes). If you only need the
  current pose at `maxTimeNs()`, prefer `getPositionLiDAR(spline->maxTimeNs(), ...)`
  which is now pure-read (Phase 1.5 Pass 3).
- When adding state to `Mapping.cpp`'s `process()` loop, hold all per-map
  mutexes via `ScopedMappingsLock maps_lock(vis_maps);` — RAII, exception-safe.
  The bare `lock_mappings()` / `unlock_mappings()` pair is gone.
- Build commands (from `rovermax_ws/` inside the container):
  ```bash
  ./scripts/colcon/colcon_build_pkg.sh resple
  ```
  CUDA builds: `colcon build --packages-select resple --cmake-args -DENABLE_CUDA=ON`.

### Sanitizer builds (recommended after substantial concurrency edits)

```bash
# TSan: catches data races. Worker + executor + async lambda + ikd-Tree
# rebuild thread are all in scope.
./scripts/docker/docker_exec.sh \
    bash -lc 'colcon build --packages-select resple \
              --cmake-args -DENABLE_TSAN=ON -DENABLE_NATIVE_ARCH=OFF'

# ASan + UBSan: catches UAF, OOB, signed overflow. Especially useful for
# verifying lifecycle deactivate/cleanup paths and ikd-Tree concurrent access.
./scripts/docker/docker_exec.sh \
    bash -lc 'colcon build --packages-select resple \
              --cmake-args -DENABLE_ASAN=ON -DENABLE_NATIVE_ARCH=OFF'
```

Run a representative bag and pipe stderr to a file. Any sanitizer hit prints
the offending stack — those are gold and almost always reveal a real bug,
not a false positive. See `HARDENING.md` Phase 0 for full operator workflow.
