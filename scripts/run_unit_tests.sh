#!/usr/bin/env bash
# Build and run the dependency-light estimator-core unit tests WITHOUT ROS 2.
#
# These cover the pure-math cores (plane fit, Joseph-form covariance, NIS,
# generic PointCloud2 field parsing) and need only a C++17 compiler, Eigen and
# GoogleTest — so they run on a laptop or in a minimal CI image in seconds.
#
#   sudo apt-get install -y libeigen3-dev libgtest-dev cmake g++
#   ./scripts/run_unit_tests.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-${ROOT}/build/unit-tests}"

cmake -S "${ROOT}/resple/test" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${BUILD_DIR}" -j"$(nproc)"
ctest --test-dir "${BUILD_DIR}" --output-on-failure
