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

What they guard: plane-fit recovery + degenerate cases, Joseph-form covariance
(symmetry/PSD), NIS, and generic PointCloud2 field resolution / multi-datatype
& endianness reads / time normalization.

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

## 3. Sanitizer bag-replay — memory/thread-safety, run manually

`scripts/run_sanitizer_replay.sh <asan|tsan> <bag> [launch_file]` builds the
package with ASan/TSan and replays a bag through the node, failing on any
sanitizer report. Needs a workspace + a recorded dataset.
