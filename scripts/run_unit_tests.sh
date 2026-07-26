#!/usr/bin/env bash
# Build and run the dependency-light estimator-core unit tests WITHOUT ROS 2.
#
# These cover the pure-math cores (plane fit, Joseph-form covariance, NIS,
# generic PointCloud2 field parsing) and need only a C++17 compiler, Eigen and
# GoogleTest — so they run on a laptop or in a minimal CI image in seconds.
#
#   sudo apt-get install -y libeigen3-dev libgtest-dev cmake g++
#   ./scripts/run_unit_tests.sh
#
# If PCL is also installed, the ikd-Tree concurrency regression
# (test_ikdtree_concurrency, HARDENING Phase 2.4) is built and run too. For the
# strongest signal run it under ThreadSanitizer with the suppressions file:
#
#   cmake -S resple/test -B build/tsan \
#     -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1 -fno-omit-frame-pointer"
#   cmake --build build/tsan -j"$(nproc)"
#   TSAN_OPTIONS="suppressions=$PWD/resple/test/tsan_suppressions.txt" \
#     ctest --test-dir build/tsan --output-on-failure -R IkdTreeConcurrency
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-${ROOT}/build/unit-tests}"

cmake -S "${ROOT}/resple/test" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${BUILD_DIR}" -j"$(nproc)"
# WHY --no-tests=error: plain `ctest` exits 0 when it finds ZERO tests, so this
# gate passed vacuously if test discovery silently produced nothing — a renamed
# test source (see the EXISTS guards in resple/test/CMakeLists.txt), a
# gtest_discover_tests POST_BUILD step that did not run, or a wrong BUILD_DIR.
# ctest >= 3.20 supports the flag; verified non-zero (exit 8) on an empty
# project with this ctest 3.28.
ctest --test-dir "${BUILD_DIR}" --output-on-failure --no-tests=error
