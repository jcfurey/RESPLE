# CLAUDE.md — RESPLE

Guidance for Claude Code when working inside `src/packages/localization/resple/`.
This package is vendored from the upstream ASIG-X RESPLE repo with workspace-local
modifications (lifecycle node, diagnostics, action server, lock hardening). The
upstream README covers usage and datasets; this file covers the architecture and
hazards you need to understand before editing.

## What it is

RESPLE is a recursive B-spline state estimator for 6-DoF odometry from one or
more LiDARs, optionally fused with an IMU. Four modes are supported upstream
(LO / LIO / MLO / MLIO). **On the Rover MAX 150 we currently run LIO** —
`if_lidar_only: false` in `src/settings/params/localization/resple.yaml`. The
IMU is fused into the IEKF continuously (not just for gravity init), and
Sierra consumes the resulting odometry on top of that.

Integration point in the stack:

```text
Ouster OS1-16 points (10 Hz) ─┐
Ouster IMU (100 Hz) ──────────┴─→ RESPLE (B-spline LIO) ─→ /localization/resple/odometry (10 Hz)
                                                                       │
                                                                       ▼
                                                                  Sierra OdometryPlugin
                                                                       │
                                                                       ▼
                                                       TF: odom → base_footprint
```

Sierra owns the TF. RESPLE publishes `odom` as a sensor source with
`odom/publish_tf: false`. See `src/settings/params/localization/sierra.yaml`
for the consumer side (`resple_odom` plugin, trust 0.85, heading reference).

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
`m_buff` in one transaction (`RESPLE.cpp:1575-1594`). This is the fix for a
prior crash where the callback could see `if_init_filter == true` before the
spline pointer was actually initialized. The ordering requirement is:

1. Fully construct the `SplineState`.
2. Point `spline` at it.
3. Flip `if_init_filter = true` under `m_buff`.

**Do not reorder.** The current state machine is a bool pair, not an enum —
it is fragile. A cleaner refactor (atomic state enum with explicit transitions)
is tracked in the hardening plan (see Hazards below).

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

## Build configuration

Flags are defined in `src/packages/localization/resple/resple/CMakeLists.txt`.
The workspace does **not** extend them via `colcon_defaults.yaml` — this
package opts out of the workspace's `-Wshadow -Wunused` so upstream compiles
clean.

| Flag | State | Notes |
| --- | --- | --- |
| `-O3` | On in Release | — |
| `-march=native` | On (`ENABLE_NATIVE_ARCH`) | Binary is non-portable across hosts. OK for single-host deployment. |
| `-ffast-math` | **On** | Allows reassociation, assumes no NaN/Inf. Numerical stability risk; see Hazards. |
| `-Wall` | On | `-Wextra` / `-Wshadow` / `-Wpedantic` are NOT enabled. |
| `EIGEN_INITIALIZE_MATRICES_BY_NAN` | Debug only | Release builds have uninitialized Eigen state unless explicitly zeroed in code. |
| `ENABLE_OPENMP` | On | Required for parallel findCorresp / pointBodyToWorld. |
| `ENABLE_CUDA` | **Off** | CUDA k-NN path exists (`src/gpu/cuda_knn.cu`, `RESPLE_USE_CUDA`) but is not built by default. |
| `BLAS` | Detected if present (`-DEIGEN_USE_BLAS`) | Destabilized the simpler `(I-KH)P` update historically; safe now that the Joseph-form posterior is used. |

The library target is built once and linked by both the `RESPLE` node
executable and the `Mapping` standalone executable. The component registration
is `PLUGIN "RESPLE" EXECUTABLE RESPLE_node` — we launch the executable, not
the composable node.

## Known hazards and the hardening plan

The following were identified in a dedicated audit. Treat these as the working
list for threading/memory/accuracy work in this package. File:line citations
reference the current HEAD.

### Concurrency

1. **ikd-Tree `Nearest_Search` lock contract is not explicitly documented.**
   The IEKF takes `mtx_map_` (shared) and runs `#pragma omp parallel for` over
   findCorresp, which calls `Nearest_Search` (`Association.h:58`,
   `Estimator.h:408`). Shared-lock concurrent calls are only safe if the ikd-Tree
   internally serializes its own tree-mutation vs. search operations. Needs
   verification in `ikd_Tree.cpp` + a stress test before relying on it.
2. **Initialization state machine is a bool pair.** `if_init_filter` +
   `if_init_map` + `localmap_initialized_` — the ordering rules are correct
   today but easily broken by future refactors. Should become a single
   `std::atomic<State>` enum.
3. **`map_update_future_.valid()` is read without a lock** in `on_deactivate`
   (`RESPLE.cpp:222`) and `on_cleanup` (`RESPLE.cpp:259`). libstdc++ makes
   this safe in practice, but the standard does not guarantee it.

### Memory and lifecycle

4. **Unbounded knot growth.** `SplineState::t_knots` / `q_knots` / `ort_delta`
   grow indefinitely; there is no pruning. A long run eventually triggers
   vector growth reallocation (fine) plus unbounded memory (not fine).
5. **Unbounded input buffers.** `pc_buff` and `imu_int_buff` have no max size.
   If the worker stalls, memory grows without backpressure or drops.
6. **`assert()` in hot paths.** `SplineState.h:108, 130, 227` abort the
   process on bad knot indices. An IEKF that produces an out-of-range query
   should degrade, not crash.

### Accuracy

7. **No divergence detection.** Covariance trace, λ_min, and NaN checks on
   the posterior would catch filter collapse before the bad pose propagates
   downstream. Currently the node silently publishes garbage if IEKF fails.
8. **Plane-fit rejection is incomplete.** `Association.h:59-68` uses only a
   point-distance threshold (`pd2 < 5`) and hardcoded `esti_plane` threshold
   `0.1f`. No eigenvalue-ratio test, no counters for dropped candidates.
9. **Deskew uses post-IEKF spline.** The spline is refined by the current
   IEKF pass, then `pointBodyToWorld` queries the *refined* knots. This is
   intentional iterative refinement, not a bug, but it is undocumented and
   depends on the scan timestamps lying within the knot window. Out-of-window
   queries currently hit `itpPose` without bounds checking.
10. **`-ffast-math` is on.** Allows the compiler to assume no NaN/Inf and to
    reassociate FP operations. For a Kalman filter doing 24×24/30×30 Joseph-form
    updates this is risky; it should be evaluated against drift metrics.

### Ordering for fixes

See the remediation plan in the session that produced this doc:
instrument first (Phase 0: TSan/ASan/UBSan + knot-growth logging) before
changing code, then work through safety fixes → concurrency → accuracy →
diagnostics. Do not skip Phase 0; most items above are suspected, not
confirmed triggered.

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

The `cube_len` hardcoded default in code (`RESPLE.cpp:580`, value 2000) is
overridden by the param; check the logged value on startup rather than reading
the source.

## Working with this package

- **Don't modify the upstream files under `config/` or `launch/`** (dataset
  variants) — our runtime uses `settings_erdc` + our YAML. Upstream files are
  kept for reference and occasional benchmarking.
- **Do modify the workspace config** at `src/settings/params/localization/resple.yaml`.
- When adding a new sensor type, wire the callback through `sensor_cb_group`
  (not the default group) and push to a per-sensor `LidarData` struct with its
  own `mtx_pc` — do not add heavy work inline in the callback.
- When adding shared state read by `processData` and any callback, decide
  which existing mutex covers it. Do not introduce a fifth mutex without
  updating the lock-ordering rule above.
- Build commands (from `rovermax_ws/` inside the container):
  ```bash
  ./scripts/colcon/colcon_build_pkg.sh resple
  ```
  CUDA builds: `colcon build --packages-select resple --cmake-args -DENABLE_CUDA=ON`.
