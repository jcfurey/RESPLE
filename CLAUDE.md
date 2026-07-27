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

### Leaf locks (outside the four-mutex ordering)

A few auxiliary mutexes exist beside the four above. Each is a **strict
leaf**: you may take it while holding any of the four, but you must never
acquire ANY other lock while holding it. Current leaves: `static_br_mutex_`
(serializes `static_br_->sendTransform` between the sensor-callback LiDAR
latch and the worker's IMU latch; note the IMU-callback path reaches it
while HOLDING `m_buff` — that nesting is legal, m_buff→leaf, but it means a
one-shot DDS write happens under `m_buff`), `save_map_mutex_` (SaveMap
thread handoff), and the TF-ownership monitor's internal mutex
(`tf_ownership.h`, fully self-contained). When adding a mutex, either slot
it into the ordering above or document it here as a leaf — a lock that is
neither is where the next ABBA deadlock comes from.

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
| 5   | Regression tests                   | done except bag-gated ACCURACY test (CI: ROS-free + ASan/UBSan + ikd-Tree TSan + the 2026-07-11 bag-free `e2e-smoke` job — full node driven by `data_injector`'s stationary scene, pose must hold the origin; `scripts/overload_rehearsal.sh` covers the bag A/B manually) |

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
| 7  | No divergence detection | **fixed** (NIS detector + `nis_recovery_mode` off/hold/reset; hold gates odom/TF while DIVERGED, reset reinflates the IEKF covariance to `nis_reset_cov`) | 0 → 3.3 |
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
| 69 | Unbounded processing **latency** under CPU overload (distinct from hazard 5's memory bound): the worker drains the whole scan backlog per pass, so on a starved machine the pose output lags, then bursts — and after any drop `propRCP` extrapolated constant-velocity across the gap with no horizon, yanking the pose on re-entry (and tripping the IMU residual gate via garbage `imu_itp`) | **capability landed, default off** (2026-07-07 overload hardening: `max_latency_ms` data-time shedding in `pushScanBounded` via `utils/overload_control.h`; `gap_extrap_decay` damped propRCP re-entry; `executor_threads` cap; `publish_current_scan`/`publish_est_window` gates; Mapping `map/max_scan_buffer` counted drops + `map/publish_min_interval_ms` batch publish; overload block in the Diagnostics msg: `rt_factor`/`backlog_ms`/`shed_scans`/`cycle_overruns`/`gap_extrap_knots`. One deliberate default change: `num_threads` clamps to `hardware_concurrency−2` at runtime. Profile: `config_low_resource.yaml`; see PARAMETERS.md "Resource-limited machines") | 2026-07-07 |
| 70 | No guard against a FOREIGN publisher on the TF pair each node owns (RESPLE `odom→frame_id`, Mapping `map→odom`): tf2 has no ownership concept and the ROS 2 wire message carries no publisher identity, so a second owner (robot_localization odom EKF on the same `odom→base_footprint`, DLIO, a bag-replayed `/tf`, a stray static publisher) interleaves silently and the pose flickers — the production master-launch hard-fail guard lives downstream, not in this repo. Two-parent variant: EKF `odom→base_footprint` + URDF `base_footprint→base_link` + RESPLE `odom→base_link` gives `base_link` two parents, broken tree with no same-pair collision | **detect-only guard landed** (2026-07-07: raw `/tf`+`/tf_static` watchers in both nodes; self-vs-foreign by exact-stamp matching in `utils/tf_ownership.h` — DDS loops own publications back, so the ring of self-published stamps is the discriminator, Mapping's static latch whitelisted forever; `tf_conflict_action` warn(default)/yield, `tf_conflict_quiet_sec`, `tf_absent_warn_sec` absence check for the demoted-but-no-EKF case; Diagnostics msg `tf_foreign_same_pair`/`tf_foreign_other_parent`/`tf_conflict_active`/`tf_yielding` + `/diagnostics` WARN/ERROR escalation; coexistence recipe in PARAMETERS.md "TF ownership") | 2026-07-07 |
| 71 | RESPLE `have_imu_transform_` (a node member) survives `on_cleanup` while `on_configure` resets `imu_to_baselink_` to a zero-quaternion default → a `deactivate→cleanup→configure→activate` re-cycle skips the IMU-extrinsic re-lookup and fuses the IMU (and gravity init) through the bogus zeroed transform, silently | **fixed** (reset the latch in `on_cleanup`) | 2026-07-14 |
| 72 | Mapping `tf_buffer`/`tf_listener` (node-level, created every `on_configure`) never reset in `on_cleanup` → a listener thread leaks per re-configure, and the next `on_configure` reassigns `tf_buffer` before `tf_listener`, briefly pointing the old listener at a freed buffer (UAF if a `/tf` callback fires mid-reconfigure). RESPLE already reset both; Mapping didn't | **fixed** (reset listener-then-buffer in `on_cleanup`) | 2026-07-14 |
| 73 | RESPLE `diagnostics_.add("System Health")` runs in `on_configure`; `diagnostic_updater::Updater::add` appends unconditionally → each re-configure cycle stacks a duplicate task (all invoked per update). Mapping avoids this by registering in its constructor | **fixed** (one-shot `diag_registered_` guard) | 2026-07-14 |
| 74 | Mapping `/global_map` published with `header.stamp = 0`: `transformCloud` fills only points (the reused cloud's PCL header stays 0) and `publishMap` set only `frame_id` → consumers doing a stamped TF lookup resolve at t=0 (no TF there) and drop the cloud unless their fixed frame is exactly `map` | **fixed** (stamp with `node_handle_->now()`, sim-time aware) | 2026-07-14 |
| 75 | `static_br_->sendTransform` (publish_extrinsic_tf mode) raced: the LiDAR latch (`latchExtrinsicTf`) runs on the sensor callback group, the IMU latch (`latchImuExtrinsicTf`) on the WORKER during `initialization()`, and `StaticTransformBroadcaster` mutates its transform `std::vector` with no internal lock → concurrent `push_back`/read corrupts `/tf_static`. Gated to `tf_extrinsics=false`+`publish_extrinsic_tf=true`+`q_ib/t_ib` (production never creates `static_br_`) | **fixed** (leaf `static_br_mutex_` around both broadcasts) | 2026-07-14 |
| 76 | RESPLE `map_update_pending_`/`map_update_future_` not reset in `on_cleanup`, and the worker's map-update `wait_for` was unguarded by `valid()` → if a map-update future was abandoned in `waitForMapUpdateBounded` (>2 s stall) leaving the future moved-from and the pending flag stuck true, the next activation's worker throws `std::future_error` every cycle (caught+retried) → map updates never resume while odom keeps publishing | **fixed** (reset both in `on_cleanup` + `.valid()` guard on the worker wait) | 2026-07-14 |
| 77 | An async map-update lambda abandoned in `on_deactivate` keeps running and iterates `lidars` (in `lasermapFovSegment`, under `mtx_map_`) while `on_cleanup` destroys the container → iterator UAF | **fixed** (clear `lidars`/`lidars_data` under `mtx_map_` in `on_cleanup`; the worker is already joined) | 2026-07-14 |
| 78 | `est_window` writer depth `QoS(50)` ≪ Mapping subscriber `QoS(10000)`: RELIABLE bounds loss only up to the WRITER's history, so a Mapping stall > ~0.5 s (at knot_hz) dropped est_windows at the writer that the replica cannot recover → replica knot-time-axis skew | **fixed** (writer depth raised to 2000, matching Mapping's app-level `est_window_queue_` cap) | 2026-07-14 |
| 79 | RESPLE LIO retained the pre-init IMU samples (`>= start_t_ns`) at init while LO cleared them — but `getImuCallback` stages pre-init IMU RAW (sensor frame) and applies the base_link extrinsic (`transformImu`) only POST-init. So the first ~scan of LIO IMU was fused in the sensor frame, a systematic accel/gyro-direction error at startup scaling with the IMU mounting rotation (finding #9 fixed only the gravity-init mean, not the per-sample data). Production Ouster IMU is mounted rotated | **fixed** (clear `imu_int_buff`+`imu_buff` in LIO too at init; fresh transformed samples resume on the next callbacks) | 2026-07-14 |
| 80 | Generic `PointCloud2` path: `genericLidarCallback` gated `isfinite` on x/y/z but not the per-point time (`.intensity`) → a non-finite time-field value made `ms2ns(NaN)` a UB cast and, once buffered, poisoned the deskew `std::sort` comparator (NaN breaks strict-weak-ordering → UB/crash). Hand-written loaders drop it via their `intensity>=0` gate; the generic path didn't | **fixed** (gate `isfinite(pt.intensity)` too) | 2026-07-14 |
| 81 | `lidar_time_offset` was applied only in the Ouster and generic callbacks; Livox/Livox2/AVIA/Hesai/Mid360Boxi set `time_begin` from the raw header stamp with no offset → the LiDAR-vs-IMU/spline time base correction was a silent no-op for 5 of the 7 sensor types | **fixed** (subtract `time_offset` in all callbacks; default 0 = no-op) | 2026-07-14 |
| 82 | Generic adapter: `convertCloud` bounded only `num_points*point_step <= data_size`, not each field's `offset+size <= point_step`, and `ingestPointCloud2`'s organized-cloud repack bounded `row_step*height` but not `row_step >= width*point_step` → a malformed PointCloud2 (field past the stride, or `row_step` under-declared) drove an OOB heap read in `readFieldAsDouble`/the per-row `memcpy` | **fixed** (field-fits guard in `convertCloud`; `row_step` lower-bound in `ingestPointCloud2`) | 2026-07-14 |
| 83 | `gap_extrap_decay` was assigned to `GapExtrapDamping::decay` unclamped; a negative value makes `scale(k)=pow(decay,excess)` sign-flip for odd excess → propRCP's gap extrapolation pushed the wrong way (misconfig; overload feature is default-off) | **fixed** (clamp to ≥0 with a WARN; 0 = hard hold) | 2026-07-14 |
| 84 | ikd-Tree inline `Rebuild` called `BuildTree(...,PCL_Storage.size()-1,...)` with no empty-flatten guard (benign only via the `size_t→int` underflow to a negative `r`; the two sibling rebuild sites already guard). Plus doc-vs-code drift: `overload_control.h` "never counts the last element" (it can) and `filter_health.h` NIS window labelled "sliding" (it is tumbling) | **fixed** (empty guard on the inline rebuild; both comments corrected) | 2026-07-14 |

| 85 | The hazard-81 `lidar_time_offset` fix was ONE-SIDED: RESPLE's 5 non-Ouster callbacks got the subtraction but Mapping's matching buffs kept raw stamps (only Ouster/Generic read the param there) → with a nonzero offset the two nodes disagreed on the scan time base for 5 sensor types — map smear ≈ velocity×offset, or scans dropped by the replica's window gate. The 5 RESPLE sites also lacked the `stamp_ns < time_offset` early-sim-time guard their Ouster/generic siblings pair with the subtraction | **fixed** (param read hoisted into `MappingBase` — one read, all 7 buffs shift identically; guard + subtraction added to all 5 buffs and all 5 RESPLE callbacks) | 2026-07-14 |
| 86 | The hazard-80 non-finite-time gate was also one-sided (RESPLE's generic callback only; Mapping's `GenericPC2Buff` passed NaN time/xyz to a UB `int64_t(NaN*1e6)` cast and `/global_map`), and it missed finite-but-huge values — `ms2ns(1e24 ms)` is an out-of-range float→int64 cast (UB) under the same malformed-input threat model | **fixed** (gate moved into the shared `ingestPointCloud2`: isfinite on x/y/z/time + a 10-minute relative-offset ceiling, covering both nodes in one place) | 2026-07-14 |
| 87 | Map-update zombie protocol: the hazard-76 `.valid()` guard FELL THROUGH to the unlocked bg-buffer swap + a new async launch when `pending && !valid` (deactivate→activate skips on_cleanup) — racing a still-running abandoned lambda; on_cleanup's pending reset enabled a double-lambda whose generation-blind trailing `store(false)` could mask the new one; a SECOND abandonment joined the first zombie via the single `thread_local` future slot (async future dtor joins); on_cleanup's new unbounded `mtx_map_` wait could hang forever on a wedged lambda; and a zombie acquiring `mtx_map_` after cleanup's clear could latch garbage ±1.8e308 cube vertices that persist into the next activation | **fixed** (worker treats `pending && !valid` as zombie-alive and SKIPS the map-update cycle — self-heals when the zombie's release-store lands; on_cleanup no longer clears `pending` (only the lambda does); abandoned futures parked in a reap-then-append list; cleanup's `mtx_map_` acquisition bounded (2 s try_lock loop, leak-but-safe on timeout); `lasermapFovSegment` early-returns on empty `lidars`) | 2026-07-14 |
| 88 | Mapping `on_cleanup` reset publishers/broadcasters/tf_buffer without checking whether the worker actually exited — `joinProcessingThreadBounded` DETACHES a >2 s-wedged worker (canonically stuck in `canTransform` under a frozen sim clock), which still dereferences them in `pubOdom` → shared_ptr race + UAF into a destroyed `tf2_ros::Buffer`. The old leak accidentally kept the detach path memory-safe | **fixed** (gate the resets on `processing_thread_exited_`; deliberate leak-but-alive fallback with an ERROR log when detached) | 2026-07-14 |
| 89 | Post-review tail: `have_imu_transform_` was reset only in `on_cleanup`, which the lifecycle ERROR path (ErrorProcessing→Unconfigured) skips — hazard 71 re-manifested on error recovery; `/global_map` stamped `now()` (publish time) sits AHEAD of every TF on the live timeline → stamped lookups fail with extrapolation-into-the-future (mirror of the stamp-0 bug); `itpQuaternion`'s `if(J_w)` branch dereferenced `w_out` unguarded and the w_out fix had triplicated the angular-velocity recursion; the pre-start IMU trim was dead code under the hazard-79 clear with a stale "LO mode" comment inviting its reintroduction; the fifth mutex wasn't in the lock-ordering section its own rule requires | **fixed** (latch reset duplicated into `on_configure` beside the transform reset; `/global_map` stamped with batch data time; `if (J_w \|\| w_out)` hoist — one recursion copy, guarded value write; dead trim deleted + comments corrected; "Leaf locks" subsection added to the Locking section) | 2026-07-14 |

| 90 | `getSplineMsg` shipped the sender's OWN idle slots (absolute knots `pruned-3..pruned-1`) as the est_window anchor, but a fresh receiver rebuilds its idle slots from them and then chains EVERY knot quaternion off `q_idle[2]` (`addOneStateKnot`: `q_knots[0] = q_idle[2]·exp(ort_delta[0])`) — so `q_idle[2]` must be the absolute orientation of knot `start_idx-1`. Those coincide only while `start_idx == num_knots_pruned_`; at startup RESPLE is unpruned (pruned=0) while `start_idx` has walked up to the handshake cost T (the receiver rejects every window until `if_init_succeed`, and the `last_start_idx_+1` throttle advances `start_idx` one knot per publish meanwhile). The replica was therefore short the rotation accumulated over the skipped knots 0..T-1. Because a window carries absolute POSITIONS but only orientation DELTAS, the position error decays after two knot intervals while the orientation error is **permanent** — every later knot chains from the mis-anchored knot 0, so `/global_map`, `traj_path` and Mapping's odom stay rotated for the whole session. Measured end-to-end through the real `getSplineMsg`: **28.6° persistent** yaw error and 0.41 m transient position error for a 0.5 s handshake at 0.57°/knot and 1 m/s. Gated on the platform MOVING during the handshake, so the default `init_attitude_source: gravity` (blocks until stationary) is safe; `tf`/`tf_gravity_check` (documented "inits while moving") and a RESPLE respawn mid-motion reach it | **fixed** (ship the three knots immediately PRECEDING `start_idx` as the anchor, with `start_q` = absolute quaternion of knot `start_idx-3`; falls back to the idle slots when `start_idx < pruned+3`, where they ARE the correct anchor — no wire-format change, and identical to the old content whenever `start_idx == pruned`). Regression: `test/test_est_window_anchor.cpp` drives the real `getSplineMsg` and reconstructs the receiver exactly as `getEstCallback` does; the two moving cases fail pre-fix (28.6°/0.41 m) and pass after, the stationary case passes both | 2026-07-26 |

| 91 | `/odom` + `/pose_cov` reported the pose ORIENTATION covariance in the BODY frame while `geometry_msgs/PoseWithCovariance` specifies a **fixed-axis** representation (axes of `header.frame_id`, i.e. `odom`). `getLastPoseCovariance`'s `G` was `2·rows1..3 of Qleft(q⁻¹)` — the right/local perturbation `δφ_b = 2·imag(q⁻¹⊗δq)` — while the message contract wants `δφ_w = 2·imag(δq⊗q⁻¹)`. `robot_localization` rotates an incoming pose covariance only from the message frame into its world frame, which is IDENTITY here (frame_id is already `odom`), so it consumed the body-frame block verbatim as world roll/pitch/yaw: any anisotropy landed on the wrong axes (over-trusting the least-observable one) and the position↔rotation cross blocks mixed frames. The position 3×3 was, and remains, correct (world). Impact is nil at identity attitude and grows with the yaw/tilt of the platform | **fixed** (`resple::geom::quatPerturbToWorld`, the `Qright(q⁻¹)` form, in the unit-tested `utils/geometry_core.h`; `P_world = R·P_body·Rᵗ`, so using the world map fixes the rotation block AND the cross blocks in one step). Regression: `test_geometry_core.cpp` `PoseCovMaps.*` pins `G_world = R·G_body`, that each map recovers the perturbation applied on its own side, and that the frame choice is observable in an anisotropic variance | 2026-07-26 |
| 92 | `estimate_msgs/Estimate.pose_covariance` is documented in the `.msg` as "6x6 IEKF posterior covariance for the last knot" but was assigned **nowhere** in the repo — it shipped 36 zeros, i.e. *perfect certainty*, to any subscriber that trusted it (a fusing consumer computes gain ≈ 1 and collapses its own covariance). `Estimate.header` likewise shipped stamp 0. In-repo harmless (Mapping never reads either), so the exposure is external subscribers | **fixed** (populated from the same posterior `/odom` publishes, under `spline_mutex_`; header stamped with the spline-edge DATA time, consistent with `/odom` and `/current_scan`). Note the est_window copy deliberately carries the RAW filter posterior — it omits the loc-gate advisory inflation `/odom` adds | 2026-07-26 |

| 93 | Estimator/robustness tail from the 2026-07-26 math re-review (core math verified CORRECT — 22/22 finite-difference checks against the real `prepLiDAR`/`prepIMU`/`update`): `prepIMU` wrote the bias columns (`BA_OFFSET`=24/`BG_OFFSET`=27) with no `if constexpr (XSIZE==30)` guard, out of range on an `Estimator<24>` (unreachable today — only the LIO estimator takes the inertial path — but a silent out-of-range write under `-DNDEBUG`); `itpPose`'s defensive early-returns cleared `J_q` while leaving `J_p` sized, and `prepLiDAR`/`prepIMU` walk the two in LOCKSTEP → `std::vector` OOB read; the "avg IEKF iterations" diagnostic accumulated the `n_iter` PARAMETER (constant 1) instead of the actual count, under-reporting ~2× (with `n_iter=1` the loop exit needs `t>1`, i.e. two associations and two `update()` calls); a missing/mistyped per-lidar `w_pt` fell back to `1e-9` = σ 31 µm ⇒ `R_inv` 1e9, i.e. effectively infinite LiDAR trust from a config typo | **fixed** (`if constexpr` guard; `J_p` cleared in lockstep on both `itpPose` return paths so a degenerate query yields a zero H row; `Estimator::lastIterations()` accessor wired into the diagnostic; `w_pt` fallback 0.01 — the value every shipped config uses) | 2026-07-26 |

| 94 | **IMU samples were fused more than once.** `pt_meas` is cleared after every frame; `imu_meas` never was, and the pre-update trim only drops samples older than `maxTimeNs − dt`, so every staged sample still inside the newest knot interval was re-fused on the following pass. The spline advances only when `pt_min_time > maxTimeNs` (`collectMeasurements`), so a knot interval holding more than `num_points_upd`=300 points takes several batches against the SAME edge — and each batch re-fused the same 6 IMU rows, with a residual the first fusion had already absorbed. The state barely moves; the posterior tightens as if a second independent sample had arrived. Effect: the IMU carries up to ~2× its configured information on dense scans, i.e. a systematically **over-confident** covariance on exactly the directions the IMU observes — published straight into the consumer EKF, which weights by inverse covariance. Also self-inflicted: a batch that drained 0 points but retained IMU ran an IEKF that broke out with `num_tot_eff==0`, feeding the §3.3 NIS detector a NaN breach for a frame that had nothing wrong with it | **fixed** (consume-once: clear `imu_meas` after a fusion that actually ran, gated on the new `Estimator::lastUpdatePerformed()` so a zero-correspondence frame keeps its unfused samples for the next pass; the trim stays as the out-of-window guard) | 2026-07-26 |
| 95 | **Process noise was injected per IEKF call, not per unit time.** `propRCP(t)`'s no-advance branch (`maxTimeNs() >= t`, i.e. the state window is UNCHANGED — a zero-time propagation) did `cov_rcp += cov_sys` unconditionally. Combined with hazard 94's multi-batch case, the process noise charged per knot interval scaled with the number of batches that happened to land in it — and that count is set by `num_points_upd` versus the scan's point density, hence by scene geometry and CPU load. Two batches per interval doubled the injected noise, so the same bag ran with a different prior/measurement balance at a different point density: a silent, load-dependent retune of the filter, in the direction of under-trusting the prior exactly when the scene is dense | **fixed** (inject at most once per distinct spline edge; bit-identical in the steady one-batch-per-knot case because the growth `propRCP` in `collectMeasurements` advances the edge first, and extra batches inside one interval now fold in sequentially with no artificial inflation between them — the correct treatment of independent measurements against one state window. `maxTimeNs()` is monotonic, so no ABA) | 2026-07-26 |
| 96 | No sanity check on the pose/twist covariance leaving the node. Every producer path is guarded (LLT failure → update skipped, NIS → divergence detector, non-finite points dropped at ingest), so a bad matrix means a bug — but `robot_localization` INVERTS the block it fuses, so one non-finite entry turns the consumer's entire state NaN permanently, with no diagnostic pointing back at RESPLE; a zero or negative diagonal (roundoff-negative variance, or an exactly-zero one at t=0) makes that inversion singular | **fixed** (`resple::health::sanitizeCovariance` in the unit-tested `utils/filter_health.h`, applied to `/odom` + `/pose_cov` and to `Estimate.pose_covariance`: any non-finite entry replaces the WHOLE matrix with an uninformative diagonal — a partially patched covariance is not a valid one — and non-positive diagonals are floored. Counted, and ERROR/WARN-throttled) | 2026-07-26 |
| 97 | **No outlier rejection fires at the update stage in steady state.** The accept test is `\|zp\| < nn_thresh` **\|\|** `lid_cov < var_pt·coeff_cov`; the second disjunct holds for any converged `P` when `coeff_cov > 1` (production 3.0), so `nn_thresh` never binds — and `robust_kernel` is `none` by default, so nothing down-weights either. `findCorresp`'s `\|pd2\| < sqrt(range)/9` is the only filter, and it admits 0.35 m of off-plane error at 10 m / 0.79 m at 50 m **at full weight**, on the rows with the longest lever arm on rotation | **capability landed, default off** (`lidar_gate_sigma`: reject when `zp² > sigma²·lid_cov`, i.e. normalized by the PREDICTED innovation instead of an absolute distance, so one setting covers both the loose post-init/post-gap regime and the converged one; `lidar_gate_max_reject_frac` (0.5) disarms the gate when it would reject more than that fraction — the standard defence against a normalized gate latching in a divergence, and the reason it cannot starve the filter. Decision logic in the unit-tested `utils/lidar_gate.h`. The `cov_escape_admits` counter ships REGARDLESS of the gate, so the exposure is measurable before anyone flips it on) | 2026-07-26 |
| 98 | **The Eigen alignment pin was inert, and the default build could SIGSEGV.** `resple/CMakeLists.txt` pinned `EIGEN_DEFAULT_ALIGN_BYTES=16`, which Eigen `#define`s UNCONDITIONALLY (`ConfigureVectorization.h:179/181`, no `#ifndef`) — the same failure the comment block directly above it diagnoses for its own first attempt with `EIGEN_HAS_POSIX_MEMALIGN`. The guarded knob is `EIGEN_MAX_ALIGN_BYTES` (`:174`). Not cosmetic, because the OTHER define IS honoured: `EIGEN_MALLOC_ALREADY_ALIGNED=1` routes `aligned_malloc` to plain `std::malloc` (16-byte), so with the pin inert and `ENABLE_NATIVE_ARCH` ON (the default, and production per this file's build table) Eigen kept `EIGEN_MAX_ALIGN_BYTES` at 32/64 and emitted ALIGNED AVX stores (`vmovapd`) into 16-byte storage. The `eigen_assert` written to catch exactly this is `#if EIGEN_DEFAULT_ALIGN_BYTES==16` (`Memory.h:184`) — false under the inert pin — so the one guard was compiled out by the same bug. Reproduced: a loop allocating and RETAINING `MatrixXd(30,30)` (the shape of `H_buf_` / the Joseph `I_X`) segfaults at `-march=native` AND `-march=x86-64-v3`, survives with `EIGEN_MAX_ALIGN_BYTES=16`. **The reproduction requires the allocations retained** — a destroy-each-iteration loop reuses one chunk that is 64-aligned by luck and hides the fault (a first probe did exactly that and wrongly came back clean). SECOND defect, same fix: the pin is PUBLIC but `-march=native` is PRIVATE to the library, so the library TU resolved `EIGEN_MAX_STATIC_ALIGN_BYTES`=64 while the executable TUs resolved 16 — measured `sizeof(struct{double; Matrix4d;})` 192/align-64 vs 144/align-16, a cross-TU **ODR violation inside one binary**. No in-repo build path reproduced either: `build_workspace.sh`, the Dockerfile, `e2e_smoke.sh` and `run_data_sweep.sh` all pass `-DENABLE_NATIVE_ARCH=OFF` | **fixed** (`EIGEN_MAX_ALIGN_BYTES=16`; plus `resple/cmake/eigen_align_check.cpp` static_asserted at configure time with the same defines and `-march`, so a third inert pin cannot ship — verified non-vacuous: it fires on the old pin and on no pin) | 2026-07-27 |
| 99 | **The only end-to-end estimator gate in CI passed with the LiDAR measurement update dead.** `scripts/e2e_smoke_check.py`'s seven criteria are all satisfied by a node that fuses nothing: on a stationary scene, dead reckoning is bit-identical to the ground truth the gate checks. Reproduced by running with `-p nn_max_sq_dist:=1e-9` (the k-NN gate rejects every neighbour, so zero point-to-plane rows reach the IEKF) — seven PASSes with `max\|pos\|` = `final\|pos\|` = `final\|v\|` = exactly 0.0000 and `corresp_used` = 0. The checker already received `corresp_used`, `iekf_numerical_failures` and `cov_sanitized` on every Diagnostics message and read none of them. Blind specifically to the ZERO-correspondence mode; a bug producing WRONG correspondences moves the pose and IS caught | **fixed** (asserts frames-fusing-LiDAR ≥ 100, numerical failures == 0, `cov_sanitized` == 0; verified both directions — sabotaged run now FAILs with script exit 1, healthy run passes at 5924 frames) | 2026-07-27 |
| 100 | **`lasermapFovSegment` never cleared `cub_needrm`.** It is a node member and the only pre-op statement was `shrink_to_fit()`, which trims capacity but keeps every element — so each cube move re-submitted the ENTIRE history of FOV-trim boxes to `Delete_Point_Boxes`. Upstream FAST-LIO's `lasermap_fov_segment()` opens with `cub_needrm.clear();`; that line was lost. On a revisited traverse a box recorded while trimming behind the platform on the outbound leg lies INSIDE the live cube after the turnaround, so it deletes map the filter is currently localizing against. Verified against the real vendored ikd-Tree. Needs ~3 cube shifts out (~950 m at the shipped `cube_len` 1000 / `det_range` 100) plus a 2-shift return before a stale box reaches the live cube, and k-NN recovers on the next `mapIncremental` — a one-frame degradation per cube move, not a permanent hole. The other half is unbounded O(moves²) growth of the delete list, all under `mtx_map_` UNIQUE (which blocks the IEKF's k-NN) | **fixed** (`clear()` before the `shrink_to_fit()`, plus cleared in `on_cleanup` so a lifecycle re-cycle cannot carry stale boxes into a fresh cube) | 2026-07-27 |
| 101 | **`plane_min_cond_ratio` measured distance from the odom origin, not planarity.** The §3.2 guard ran a rank-revealing-QR test on the UNCENTRED scatter `Σ p pᵀ`, which is dominated by the centroid outer product `k·c·cᵀ` — so its verdict scaled as ~1/R² with distance from the origin. Measured: the good-patch pivot ratio falls ~5 decades from 1 m to 200 m, and beyond ~20 m good and noisy-collinear patches are identical to three digits. Worse, at `0.05` — the one value any shipped config used (`config_narrow_tunnel.yaml`) — it accepted **0% of ALL patches at every range**, i.e. that profile could not fuse a single correspondence and would have produced exactly the dead-filter mode hazard 99's gate was blind to | **fixed** (test moved to the CENTRED scatter `Σ(p−c)(p−c)ᵀ`, which is translation-invariant; the test also had to INVERT — a good planar patch is rank 2 there, so the discriminator is the in-plane eigenvalue ratio λ2/λ1 above a threshold, not rank == 3. Now a normalized ratio in (0,1]. Measured flat 1–200 m at `num_match_points: 5`: 0.05 → 90%/55% good/collinear, 0.10 → 82%/24%, 0.20 → 62%/8%. Tunnel profile moved to 0.10. Regressions `FitPlane.PlanarityGuardIsTranslationInvariant` / `.PlanarityGuardScaleIsNormalized`) | 2026-07-27 |
| 102 | Tail from the same pass: `SaveMap` flattened into the ikd-Tree's shared `PCL_Storage` scratch WITHOUT clearing it first (`flatten` appends; the two CUDA refresh sites do clear), so the exported map carried stale points and grew on every invocation; `itpQuaternion`'s `if (J_q)` branch still dereferenced `q_out` unguarded — the last hole in the write-every-non-null-out-param contract hazards 0 and 89 closed at the other two sites; `updateLiDARInertial` materialized a 4608-byte copy of the 24×24 prior covariance **per LiDAR point** to form a 1×1 quadratic form, loop-invariant and not hoisted by the compiler (measured 449.9 → 423.0 µs on a 300-point call; hoisting beats the block expression, 68.2 → 60.1 → 57.1 µs, and keeps the arithmetic bit-identical, which matters because `lid_cov` feeds the accept test and the hazard-97 gate); both assemblers used `conservativeResize` on three buffers that are `setZero`'d on the next line | **fixed** | 2026-07-27 |

**2026-07-27 full-repo audit (hazards 98–102):** an 11-area multi-agent audit
(core estimator, spline, association, worker loop, ikd-Tree algorithm; Mapping,
utils, CUDA, config/launch, test suite, build/packaging) with adversarial
verification of every finding — 36 raised, 30 confirmed, **15 refuted**. The
refutation rate is the point: several plausible, well-argued performance and
correctness claims did not survive contact with a measurement (a claimed
`removeNaNFromPointCloud` no-op, a scan-drain copy, a CUDA `cudaError_t`
discard, an ODR claim about the test targets).

Two findings interlock and are worth remembering together: hazard 101 made the
narrow-tunnel profile reject every correspondence, and hazard 99 meant the
end-to-end gate could not see a filter that fuses nothing. A broken config and
a blind gate for the same failure mode.

Known limit recorded rather than fixed (pinned by
`FitPlane.FitPlaneFarFromOriginLimit`): `fitPlane` solves `A·n = −1`, so its
conditioning is ~(|centroid| / patch extent)² and it degenerates a few km from
the ODOM ORIGIN — exact to ~1e-13 at 1500 m, total failure by 2000 m, where
ColPivHouseholderQR returns a least-norm vector and the residual gate rejects
every fit. The odom origin is the start pose and is never reset, so a
multi-kilometre traverse walks into it; the failure mode is starvation, not
mis-estimation. Fixing it means re-parameterizing the fit around the patch
centroid, which changes the numerics of the whole correspondence path and wants
bag validation.

**2026-07-26 accuracy pass (hazards 94–96):** driven by "anything that produces
accuracy should be prioritized", a pass over the measurement-weighting path
rather than the crash/lifecycle surface. The two behaviour-affecting fixes both
concern **information accounting**, and both were invisible to the earlier
audits because they only bite when a knot interval needs more than one
measurement batch (dense scan vs. `num_points_upd`) — a regime no unit test
reaches and no single-frame reading exposes. They pull in opposite directions
(94 over-tightens the posterior, 95 over-loosens the prior) and are therefore
landed together, not separately: fixing one alone shifts the tuning.

Hazard 97's gate was validated on the running node (`scripts/e2e_smoke.sh`,
which now takes `RESPLE_EXTRA_ARGS` for exactly this), not just in unit tests:

- default (`lidar_gate_sigma: 0`) and armed (`5.0`) runs both pass the
  stationary-scene criteria with the same 1.3–1.4 mm final drift, and with the
  gate armed on clean data it rejects **nothing** (174/174 correspondences used)
  — no false positives to pay for.
- `cov_escape_admits` is a real discriminator, not a constant: 0 on the clean
  synthetic scene (where `nn_thresh` genuinely binds), **2 516 303** in the same
  run with `nn_thresh: 0.001` forcing every row through the escape disjunct.
- the non-starvation property holds on the real filter, not only in the unit
  test: with an absurd `lidar_gate_sigma: 0.0005` the gate flagged most rows on
  **10 468 consecutive updates and disarmed on every one** — `rejected` stayed
  exactly 0, NIS stayed healthy (0.81×dof), the node never diverged.

Deliberately NOT changed, with the reasoning recorded so it is not re-litigated:

- **The update-stage LiDAR gate is inert in steady state.** `updateLiDAR*`
  accepts a row when `|zp| < nn_thresh` **||** `lid_cov < var_pt·coeff_cov`.
  With `var_pt`=0.01 and `coeff_cov`=3.0, `lid_cov = H·P·Hᵗ + var_pt ≥ 0.01`
  is below 0.03 for any converged `P`, so the second disjunct short-circuits
  and `nn_thresh` never binds. `coeff_cov: 1.0` makes the second disjunct
  unsatisfiable (it would need `H·P·Hᵗ < 0`) and thereby *activates*
  `nn_thresh` — which is exactly what `config_narrow_tunnel.yaml` does. The
  disjunct's direction reads backwards for outlier rejection (it admits a
  large residual precisely when the prior is CONFIDENT, which is the
  definition of an outlier), but it is upstream behaviour and flipping the
  comparison is a large, un-bagged behaviour change. Addressed additively
  instead — see hazard 97.
- **`findCorresp` is the only outlier gate that actually fires today**, and its
  test is `|pd2| < sqrt(range)/9`: 0.079 m at 0.5 m, 0.111 m at 1 m, 0.351 m
  at 10 m, 0.497 m at 20 m, 0.786 m at 50 m. So close range is already tightly
  gated and `nn_thresh: 0.5` would only ever bind beyond ~20 m even if it were
  reachable. The exposure is the mid/far field — where a bad correspondence
  also has the longest lever arm on rotation.

**2026-07-26 estimator math re-review (hazard 93):** a full fresh-eyes pass over
the estimation math, verified by a probe compiled against the REAL estimator
(`#define private public`, not a transcription) — 22/22 FD checks pass. Explicit
clean bills: the Gauss-Newton step (`deltax = KH·δ + K·ν − δ` expands exactly to
the FAST-LIO iterated form, legitimate here because the RCP state is Euclidean);
`prepLiDAR` H to ≤4e-9 and `prepIMU` H to ≤1e-8 relative across the whole 4-knot
window including the partial-window truncation; the slot↔knot mapping
(`idx_window ≡ 4−size_J`, so `j<4` can never trip); `Quater::exp(v)` representing
a rotation of **2|v|** with every consumer honoring it (the ×2 in `w_itps` is the
`2·vec(A⁻¹Ȧ)` factor); blending matrices bit-identical to the standard cubic
B-spline and its cumulative sum; the interleave loop consuming every measurement
EXACTLY once (verified exhaustively, including unsorted multi-LiDAR stamps);
`propRCP`'s `2·p₂−p₀` being the exact constant-velocity extrapolation that
`a_mat` encodes; `range > 81·pd2²` ≡ FAST-LIO's `s>0.9`; `fitPlane`'s `d=+1/|n|`
sign and no measurable normal bias vs a PCA fit; PCL's `VoxelGrid`
`downsample_all_data_` defaulting TRUE so the per-point time in `.intensity`
survives downsampling as the voxel mean (deskew is NOT silently disabled);
`drot`/`drotInv` being MISNAMED but both call sites picking the right one.

Two accuracy findings left UNFIXED pending bag A/B (behaviour-affecting):
- **`nn_thresh` is inert in steady state.** The accept test is
  `|zp| < pt_thresh **||** lid_cov < var_pt·cov_thresh`; the second clause fires
  iff `H·P·Hᵗ < var_pt·(coeff_cov−1)` = 0.02 m² at production `coeff_cov: 3.0`,
  and measured `H·P·Hᵗ` is 8.9e-6 (1 m) … 7.6e-3 (30 m) — always below it. So
  `nn_thresh` does nothing under ~40 m and the only residual gate left is
  `findCorresp`'s `range > 81·pd2²` at the PRE-update pose, admitting
  mis-associations 0.35–0.79 m off-plane at full weight (`robust_kernel` is off
  by default). The clause is also backwards from intent: it disables outlier
  rejection precisely when the filter is CONFIDENT. `config_narrow_tunnel.yaml`
  already neutralizes it with `coeff_cov: 1.0`; production should likely follow.
- **IMU re-fusion.** `imu_meas` is trimmed only to `>= maxTimeNs − dt` and never
  cleared per cycle, so ~1–2 samples are fused twice against an
  already-tightened covariance — double-counted information, i.e. genuine
  over-confidence on IMU-observed directions.

**2026-07-26 covariance audit (hazards 91–92):** an end-to-end audit of the
uncertainty chain (`cov_rcp` → Joseph posterior → `getLastPose/TwistCovariance`
→ the published messages) prompted by "make sure odometry covariances are
properly updated". The chain came back largely CORRECT — no staleness (the
covariance is recomputed in the same worker iteration as the pose it
accompanies), correct row-major `[r*6+c]` packing and `[x,y,z,rx,ry,rz]`
ordering at all four publish sites, twist genuinely body-frame matching
`child_frame_id` (REP-105), the loc-gate inflation truly advisory (never added
to `cov_rcp`) and applied to the correct world-translation block, NIS `hold`
gating every pose/odom/TF publisher, and a numerical-failure cycle correctly
pairing an un-updated state with the un-updated (larger) prior. Hazards 91–92
are the two real defects. Known-latent, left UNFIXED and deliberately
documented rather than changed: `propRCP` injects `cov_sys` **per IEKF call**
rather than per elapsed time (in steady state the worker's `propRCP` always
takes the no-knot `+= cov_sys` branch, so the published magnitude scales with
cycles-per-knot — i.e. with `num_points_upd` chunking and CPU load — in the
conservative/over-report direction); `odom/dense_pub_hz` back-fills up to 1000
samples that are exact resamplings of ONE posterior, all stamped with the edge
covariance, which a consumer fuses as independent; no `allFinite()` guard on
the OUTGOING covariance; and the Diagnostics `cov_trace`/`cov_lambda_min`
fields describe the pre-inflation posterior, not what `/odom` carries.

**2026-07-26 bug hunt (hazard 90):** the est_window replica protocol was
re-examined after `fdef897` ("replica time-base") fixed the *time* half of the
startup-skip problem by modelling the skipped knots as front-pruned. That
exposed the *orientation* half: the time axis was then correct while the
quaternion chain was still anchored at the spline origin. Note the asymmetry
that made this survive earlier audits — positions are transmitted absolutely
and self-heal, orientations are transmitted as deltas and do not.

**2026-07-14 series self-review (hazards 85–89):** an xhigh-effort 10-angle ×
adversarial-verify code review of this branch's own 5-commit series found 15
defects in the fixes themselves — dominated by one-sided fixes (RESPLE patched,
Mapping missed) and teardown-boundedness regressions in the wedged-lambda
paths. All 15 applied. The load-bearing protocol change: `map_update_pending_`
is now cleared ONLY by the map-update lambda itself; every other reader treats
`pending && !future.valid()` as "zombie still alive — skip, don't swap".

**2026-07-14 ingestion/utils sweep (hazards 79–84):** a three-lens audit (sensor
data-ingestion → `PointData`/`ImuData`, the ikd-Tree *sequential* algorithm, the
newer util headers + est_window replica). The ikd-Tree core algorithm and the
util headers came back essentially clean (the auditors' clean bills:
Nearest_Search/Build/downsample/counter accounting; `dense_pub`, `tf_ownership`,
`latencyShedCount`, `pruneFrontKnots`, the est_window queue). Known-latent, left
UNFIXED as unreachable/harmless under shipped params: ikd-Tree `k<=0` and
`downsample_size==0` guards (caller/config error), the `PointType_CMP` epsilon
tie-break (upstream, negligible), the `updateKnots` idle-copy corner on a
receiver re-init while the sender is pruned (does not arise on a normal RESPLE
respawn), and `RosTimeWait` sub-ns backwards-jitter re-arm.

**2026-07-14 node/ROS 2 reinvestigation (hazards 71–78):** a three-lens
multi-agent audit (lifecycle/resources, concurrency/locking, ROS 2 wire) of
`RESPLE.cpp`, `Mapping.cpp` and the middleware surface, each finding verified
against the code before fixing. Most are lifecycle re-cycle bugs (a full
`deactivate→cleanup→configure→activate` cycle) or config-gated; `static_br_mutex_`
is a new leaf lock (outside the four-mutex ordering, like the TF-monitor mutex).

**2026-07-02 bug hunt (hazards 39–68):** a multi-agent audit against the RESPLE
paper (arXiv:2504.11580), Sommer et al. 2020 (arXiv:1911.08860), the ASIG-X
vendor point, hku-mars/ikd-Tree and FAST-LIO conventions found 30 further
defects — all fixed. Full per-finding write-ups, verdicts and outcomes in
`doc/REVIEW_2026-07-02_bug_hunt_findings.md`. The ones that change contracts
documented in this file:

- **`itpQuaternion` writes `*q_out` for every non-null pointer combination**
  (was: unwritten — i.e. caller-side garbage — when `J_q == nullptr` but
  `J_w != nullptr`; `getLastTwistCovariance` published a garbage-rotated
  velocity covariance every frame).
- **Extrinsic convention is now explicit** (`tf_extrinsics` param, default
  true): TF carries the mounting extrinsic and YAML `q_lb/t_lb` is an extra
  offset; `tf_extrinsics=false` (dataset launches) = upstream YAML-only.
  Per-LiDAR TF cache; 10 s `tf_wait_timeout` fallback instead of dropping
  scans forever; WARN tripwire when TF and YAML are both non-identity (they
  compose — a verbatim-copied TF cancels the extrinsic).
- **ikd-Tree `Push_Down` no longer takes `working_flag_mutex`** (the old
  rebuild branch blocked on it inside the child node lock — ABBA against both
  the rebuild thread and whole-op mutators). `Rebuild_Ptr` and `rebuild_flag`
  are `std::atomic`; the rebuild swap always holds `working_flag` (the
  empty-flatten path didn't); `Rebuild_Ptr` is cleared inside the
  `search_rw` unique block; rebuild flattens use `NOT_RECORD` (the
  deleted-points vectors were an unbounded leak with no consumer).
- **`initFilter`'s Q indexing fixed** (upstream bug: `bottomRightCorner` on
  the 30×30 Q put the newest-RCP noise on the bias block); bias random walk
  now explicit via `cov_bias_acc_rw`/`cov_bias_gyro_rw` (defaults preserve
  pre-fix LIO magnitudes).
- **Mapping `transformPoint` mirrors `pointBodyToWorld` exactly**; scan-end
  times come from the max per-point offset (VoxelGrid reorders and averages);
  `updateKnots` pads mid-run est_window gaps to keep the replica time axis
  aligned; `setIdles` follows the arriving-delta convention.

### Initial attitude (world←base_link)

`initialization()` no longer derives the start orientation purely from the
accelerometer. `init_attitude_source` selects: `gravity` (default, unchanged —
accel only, blocks until stationary), `tf` (base_link attitude from the TF tree
`init_attitude_frame → frame_id`; inits while moving), or `tf_gravity_check`
(TF authoritative, accel cross-checks it when a stationary window exists and
WARNs on disagreement — a free `imu→base_link` extrinsic check, surfaced as a
diagnostic delta). **Invariant: the world frame is gravity-aligned (Z up) on
every path** — TF modes take only roll/pitch (yaw stays a free gauge unless
`init_yaw_from_tf`), the attitude frame MUST be gravity-aligned, and `gravity`
is set to `[0,0,gravity_magnitude]` (default 9.81, exactly the old value). The
pure-Eigen attitude math (`r2ypr`/`ypr2r`/`g2r`) moved to `geometry_core.h`
(unit-tested: `test_geometry_core.cpp` `Attitude.*`); `CommonUtils::R2ypr/ypr2R/g2R`
forward to it.

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
| `lidar_gate_sigma` | hazard 97 adaptive outlier gate: drop a point-to-plane row when `zp² > sigma²·lid_cov` (standard deviations of the PREDICTED innovation, not metres — so it relaxes while `P` is large and tightens once converged). ~`sigma`×0.1 m at the shipped `var_pt`. 0 = off | `0.0` |
| `lidar_gate_max_reject_frac` | gate escape hatch: disarm (admit everything) when more than this fraction of the update's candidate rows would be rejected — a wrong PRIOR looks like this, and disarming is what stops a normalized gate from locking in a divergence | `0.5` |
| `map_prune_radius` | §3.4 keep only map points within this distance (m) of the pose; floored at 2×det_range; 0 = off (cube-only) | `0.0` |
| `nis_recovery_mode` | §3.3 divergence recovery: `off` (detect-only) / `hold` (gate odom+TF while DIVERGED) / `reset` (reinflate IEKF covariance to `nis_reset_cov`) | `off` |
| `max_scan_buffer` | §2.3 per-LiDAR raw-scan cap (scans, drop-oldest; 0 = unbounded) | `0` |
| `max_imu_staging` | §2.3 IMU staging cap (samples, drop-oldest; was hardcoded 2000) | `2000` |
| `max_latency_ms` | overload hardening: per-LiDAR DATA-TIME latency budget; queued scans older than this vs the newest arrival are shed (counted as `shed_scans`; 0 = off) | `0` |
| `gap_extrap_decay` | damping for propRCP's across-gap constant-velocity extrapolation (1.0 = off, literal legacy expression; <1 decays velocity toward a hold beyond `gap_extrap_free_knots`) | `1.0` |
| `gap_extrap_free_knots` | knots per propRCP call exempt from the decay (steady state adds ≤ ~2) | `3` |
| `executor_threads` | MultiThreadedExecutor pool cap, both nodes (0 = rclcpp default: one per core) | `0` |
| `publish_current_scan` / `publish_est_window` | publisher gates; est_window may only be disabled with Mapping off (its sole input) | `true` |
| `map/max_scan_buffer` / `map/publish_min_interval_ms` | Mapping: per-sensor buffer cap (was silent hardcoded 200; drops now counted) / batch-don't-drop `/global_map` interval (0 = per-scan) | `200` / `0` |
| `tf_conflict_action` | TF ownership guard, both nodes: `warn` (throttled ERROR + diagnostics) or `yield` (suspend own TF while a foreign publisher on the pair is active; topics keep flowing) | `warn` |
| `tf_conflict_quiet_sec` / `tf_absent_warn_sec` | yield-resume quiet window / WARN when `publish_tf=false` but nobody publishes the pair — one-shot for never-wired, re-armed per outage when the owner dies mid-run; foreign /tf_static claims are sticky (yield holds until re-configure) | `5.0` / `10.0` |
| `q_ib` / `t_ib` | YAML IMU extrinsic (`p_imu = q_ib·p_body + t_ib`, q_lb convention); only in `tf_extrinsics=false` mode — rotates IMU samples + gravity init through the transformImu path (was: tilted IMU silently fused unrotated). Identity = legacy pass-through | identity |

**Clock domains (2026-07-08 audit):** all *wait-for-data* timeouts and
data-facing windows (`tf_wait_timeout`, `lo_imu_wait_timeout`, init-attitude
wait, TF-guard freshness/absence, Mapping batch interval) run on the NODE
clock via `utils/sim_time_wait.h` (`RosTimeWait`: starts at the first nonzero
clock sample, re-arms on backwards jumps) — bag replay at throttled rates
with `--clock` + `use_sim_time` cannot expire them early. Wall/monotonic time
is reserved for real-machine measurements (stage timings, `cycle_overruns`,
`rt_factor`'s wall denominator, bounded joins, Mapping's publish pacing) —
when adding a timeout, pick the domain deliberately and comment it; see
PARAMETERS.md "Clock domains".

`num_threads` is clamped at runtime to `max(1, hardware_concurrency − 2)`
with a WARN (both nodes) — the shipped configs assume a big machine. The one
deliberate default-behaviour change of the overload-hardening series.

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
- When adding state to `Mapping.cpp`'s `process()` loop, work out which lock
  actually covers it before reaching for one. `spline_active_` and everything
  derived from it (`opt_old_path`, `path_t_ns_`, `est_window_queue_`) is
  covered by `m_spline`, and every mutation of it is on the worker thread
  (Option B). Per-map `MappingBase` state (`pc_L_buff` and the scratch members)
  is covered by that map's own `mtx`; take all of them together via
  `ScopedMappingsLock maps_lock(vis_maps);` — RAII, exception-safe. The bare
  `lock_mappings()` / `unlock_mappings()` pair is gone.
  **`ScopedMappingsLock` is currently unused** (2026-07-27): the publish block
  and the est_window drain both held it without touching any per-map state — a
  vestige of the pre-Option-B scheme where the per-map mutexes stood in for
  `m_spline` on spline reads. It blocked all seven sensor callbacks for the
  length of a Path publish (up to 10 000 poses) every 50 ms. The helper is kept
  because it is the correct pattern the moment `process()` does touch per-map
  state; do not re-add it "for symmetry".
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
