# RESPLE tests

Two tiers, on purpose.

## 1. Unit tests — ROS-free, run automatically

Cover the dependency-light estimator/ingestion cores (`utils/geometry_core.h`,
`utils/point_cloud_adapter.h`). They need only a C++17 compiler, Eigen and
GoogleTest — **no ROS 2, no PCL** — so they run on every push/PR
(`.github/workflows/unit-tests.yml`) and on a laptop in seconds.

```bash
# standalone (no colcon)
sudo apt-get install -y libeigen3-dev libgtest-dev cmake g++
./scripts/run_unit_tests.sh

# or inside a workspace
colcon test --packages-select resple
```

What they guard: plane-fit recovery + degenerate cases (incl. the §3.2
rank-conditioning guard), Joseph-form covariance (symmetry/PSD), NIS,
axis-aligned box subtraction (§3.4 radius map pruning), generic PointCloud2
field resolution / multi-datatype & endianness reads / time normalization,
the runtime health monitors (windowed NIS divergence detection with
hysteresis, IMU fault detection: NaN/saturation/stuck/time-jump/noise/bias),
and the B-spline state itself — `test_spline_state.cpp` proves the §3.1
sliding-window knot pruning is bit-exact over the retained window and that
the est_window sender/receiver protocol survives pruning on both sides.
`SplineState.h` needs the `estimate_msgs` headers, so the standalone build
compiles it against the field-compatible ROS-free stubs in `stubs/` (the
colcon build uses the real generated messages).

### ikd-Tree concurrency regressions (needs PCL, still no ROS)

`test_ikdtree_concurrency.cpp` builds when `libpcl-dev` is present:

- `ParallelSearchPushDownRace` — phased mutate-then-search stress guarding
  the Phase 2.4 per-node `Push_Down` locking (pre-fix TSan: ~16 races).
- `RebuildVsMutatorRace` — production `mtx_map_` discipline (shared
  searches, unique mutator) against the tree's internal rebuild thread,
  guarding the Phase 2.5 fixes (recursive whole-op `working_flag_mutex`;
  lock-order inversion). **Treat it as a canary**: any TSan report is a
  real regression, but the pre-fix window is scheduler-sensitive — see the
  tuning note in the test before changing its configuration.

The strongest signal is under ThreadSanitizer with NO suppressions
(`tsan_suppressions.txt` is intentionally empty of active entries):

```bash
cmake -S resple/test -B build/tsan \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1 -fno-omit-frame-pointer"
cmake --build build/tsan --target test_ikdtree_concurrency
TSAN_OPTIONS="halt_on_error=1:suppressions=$PWD/resple/test/tsan_suppressions.txt" \
  ./build/tsan/test_ikdtree_concurrency
```

### CI mapping (`.github/workflows/unit-tests.yml`)

| Job | What it runs |
| --- | --- |
| `estimator-core` | The ROS-free suite, plain build |
| `estimator-core-asan` | Same suite under ASan+UBSan, `detect_leaks=1` (no PCL → the long ikd-Tree stress is excluded by construction) |
| `ikdtree-tsan` | Both concurrency tests under TSan, `halt_on_error=1`, empty suppressions |

## 2. Integration tests — full PCL + ROS 2, run manually

Exercise the real `sensor_msgs/PointCloud2` → `pcl::PointXYZINormal` path and
the RESPLE lifecycle node. They link the heavy stack and are **excluded from
the automatic test set** — build them only when you have a full ROS 2 (Jazzy)
workspace:

```bash
colcon build --packages-up-to resple \
    --cmake-args -DRESPLE_BUILD_INTEGRATION_TESTS=ON
colcon test --packages-select resple        # runs the C++ integration gtests

# the lifecycle pipeline test is launch_testing-based; run it explicitly:
source install/setup.bash
launch_test src/RESPLE/resple/test/integration/test_resple_lifecycle.launch_test.py
```

- `test_pc2_ingest.cpp` — PointCloud2 → PCL conversion, decimation, blind gate.
- `test_esti_plane_pcl.cpp` — `CommonUtils::esti_plane` over real PCL points.
- `test_resple_lifecycle.launch_test.py` — configure→activate→deactivate→
  cleanup on the live node, plus a clean-exit assertion (shutdown hardening).

## 3. Live sanitizer sweeps — run manually

- `scripts/run_data_sweep.sh [tsan|asan]` — no bag needed: a C++ injector
  (`test/tools/data_injector`) publishes synthetic TF + IMU + PointCloud2 so
  the LIVE node runs its full pipeline (deskew → parallel k-NN → IEKF → map
  update → spline growth/pruning) under the chosen sanitizer. Use
  `num_threads=1` for TSan (GCC's libgomp barriers are false positives
  otherwise) — the script does this for you.
- `scripts/run_sanitizer_replay.sh <asan|tsan> <bag> [launch_file]` — same
  idea against a recorded dataset; the end-to-end confidence pass once a
  representative bag exists. Needs a workspace + the bag.
