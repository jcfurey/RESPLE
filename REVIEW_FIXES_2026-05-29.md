# RESPLE review fixes — 2026-05-29

Correctness pass over the RESPLE package (post-hardening, post–Lyrical build
migration). Scope was *new* bugs not already tracked in
[`HARDENING.md`](HARDENING.md). Seven items fixed; two more noted. Package
rebuilds clean in the container (`colcon_build_pkg.sh resple`, warnings only).

## Fixes implemented

| # | Severity | File | Issue | Fix |
| --- | --- | --- | --- | --- |
| 1 | High | `Mapping.cpp` | Double extrinsic transform → wrong published/saved map | Use `base_link→IMU` instead of re-applying `lidar→IMU` |
| 2 | Medium | `Mapping.cpp` | Path/odom freeze after a RESPLE respawn (stale window) | Worker resets local trackers on a `(re)start` signal |
| 3 | Medium | `RESPLE.cpp` | Data race on diagnostic counters (worker vs. Updater timer) | Counters made `std::atomic` |
| 4 | Low | `RESPLE.cpp`, `Mapping.cpp` | `int64_ns + 1e8` evaluated in `double` (loses precision on absolute stamps) | Integer literal `100000000LL` |
| 5 | Low | `Mapping.cpp` | `transformCloud` cleared the member cloud, not the out-param | `pc_out->clear()` |
| 6 | Low | `RESPLE.cpp` | Global `ikdtree` had external linkage in a double-compiled TU | Marked `static` (internal linkage) |
| 7 | Note | `common_utils.h` | `time_list` sorts by `.intensity` (time-by-convention) | Documented the intensity-carries-time contract |

---

### 1. Mapping published/saved a geometrically wrong map (HIGH)

`MappingBase::transformPoint` applied the **lidar→IMU** extrinsic
(`lidar.q_bl / t_bl`) to points that the sensor callbacks had *already*
pre-transformed into `base_link` (via `pcl::transformPointCloud(..., lidar_to_baselink_)`).
The B-spline pose it then applied is the IMU/body trajectory in world — the same
convention as RESPLE's own `Association::pointBodyToWorld`, which applies `q_bl`
to **raw lidar** points. So Mapping applied the extrinsic twice, offsetting and
rotating the whole `global_map` (and any saved map) by the lidar↔base_link
transform whenever the LiDAR isn't coincident with `base_link` (always, on the
rover). Live localization was unaffected — this only corrupts the Mapping node's
visualization / save-map output.

**Fix.** Precompute `baselink_to_imu_ = imu_to_baselink_.inverse()` when the TF
is first resolved, and in `transformPoint` go `base_link → IMU → world`:
```cpp
Eigen::Vector3d p_imu(baselink_to_imu_ * p_body);   // was: lidar.q_bl * p_body + lidar.t_bl
Eigen::Vector3d p_global(q_itp * p_imu + t_itp);
```
This is provably equal to RESPLE's raw-point result: the TF pre-transform and
`baselink_to_imu_` cancel, leaving exactly `q_bl * p_lidar + t_bl`.

### 2. Path/odom freeze after a RESPLE respawn (MEDIUM)

The `HARDENING.md` Phase 1.6 "stale spline window after respawn" hazard, root
cause located. `process()` gates publishing on
`spline_active_.numKnots() > num_knot`, where `num_knot` is a **worker-local**
monotonic counter; `publishPath` advances a member `path_t_ns_`. On a RESPLE
respawn, `startCallBack` re-inits the spline window from a fresh start time, but
neither `num_knot`, `path_t_ns_`, nor `opt_old_path` were reset (they reset only
on a full deactivate/cleanup). The publish gate and `publishPath`'s while-loop
then never advance → path and odom go silent.

**Fix.** A `std::atomic<bool> path_reset_` is release-stored by `startCallBack`
on every (re)start and consumed once by the worker, which resets `num_knot`,
`path_t_ns_`, and `opt_old_path`. All three remain touched only on the worker
thread (no new cross-thread races). The first start is a harmless no-op reset.

### 3. Data race on diagnostic counters (MEDIUM, low impact)

`frame_count_`, `total_computation_time_ms_`, `total_iekf_iterations_`, and
`last_process_time_` were written by the worker thread but read/reset by
`updateDiagnostics`, which also runs on the **executor thread** via the
`diagnostic_updater::Updater`'s internal 1 Hz timer (not only the worker's
`force_update`). The Updater's lock serializes the two `update()` calls against
each other but not against the worker's bare writes — a real TSan-visible race.
Impact was corrupted diagnostic metrics only, never estimation.

**Fix.** Converted to atomics (matching the existing `cached_spline_knots_`
pattern):
- `frame_count_`, `total_iekf_iterations_` → `std::atomic<size_t>`
- `last_process_time_` (`rclcpp::Time`) → `std::atomic<int64_t> last_process_ns_`
- `total_computation_time_ms_` (`double`) → `std::atomic<uint64_t> total_computation_time_us_`
  (integer µs, since C++17 has no `atomic<double>::fetch_add`)

`updateDiagnostics` snapshots all three once for a consistent computation.

### 4. `int64_ns + 1e8` evaluated in `double` (LOW)

`t + 1e8` (where `1e8` is a `double` literal) promotes the whole expression to
`double`. On absolute wall-clock nanosecond stamps (~1.7e18 > 2⁵³) this rounds
to ~256 ns granularity. Negligible against a 100 ms gate and invisible in sim,
but a latent precision bug on the real-hardware path. Replaced with the integer
literal `100000000LL` at the four `RESPLE.cpp` sites (map-update gate,
`collectMeasurements` window count/transform/drain) and the one `Mapping.cpp`
site (`publishPath` step).

### 5. `transformCloud` cleared the wrong cloud (LOW)

`transformCloud(...)` called `pc->clear()` (the member) instead of
`pc_out->clear()` (the out-param). Correct today only because the sole caller
aliases the two; a latent footgun for any future caller. Now clears `pc_out`.

### 6. Global `ikdtree` external linkage in a double-compiled TU (LOW)

The Lyrical build migration compiles `RESPLE.cpp` into **both** `libresple.so`
and the standalone `RESPLE` executable. With external linkage there were two
definitions of the file-scope global `ikdtree`, and which one each function
bound to relied on ELF symbol interposition. Marked `static` (internal linkage),
matching the adjacent `g_cuda_map`. The global is only ever referenced from
within `RESPLE.cpp` (the `ikdtree` tokens in `Estimator.h` / `Association.h` are
function parameter names, not this global), so this is safe.

### 7. `time_list` sorts by `.intensity` (NOTE)

`CommonUtils::time_list` — the per-scan deskew comparator — compares
`.intensity`, which is correct only because every sensor loader overloads
`.intensity` to carry the per-point time offset (ms) and moves real reflectivity
into `.curvature`. Added a comment documenting the contract so a future sensor
path that leaves genuine intensity in the field doesn't silently break deskew
ordering. (No behavior change.)

---

## Worth noting (not changed)

- **Double-compilation of `RESPLE.cpp` / `Mapping.cpp` remains.** Fix #6 removes
  the *correctness* fragility, but the executables still re-compile the full
  ~90 KB sources that already live in `libresple.so` — a build-time cost and a
  duplicate-symbol layout that works only because one copy sits in a `.so`. The
  clean design is to keep `main()` in its own small TU and link the library,
  rather than re-compiling the whole file with `#ifndef RESPLE_LIB_BUILD`.
  Deferred — it's a CMake restructure, not a bug.

- **Diagnostic window accounting is best-effort.** Counters are read then reset
  in `updateDiagnostics`; a worker increment landing between the two is dropped
  from that window. This matches the pre-existing design and is acceptable for
  diagnostics — the atomics just make it well-defined rather than UB.

- **Pre-existing build warnings are untouched:** `transform_broadcaster.h`
  deprecation, unused `respleCrashHandler` / `mappingCrashHandler` (installed by
  address, intentionally), and the Eigen AVX `-Warray-bounds` noise in
  `Estimator::prepLiDAR` (a known false positive from a `1×3` block assign).
