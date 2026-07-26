# Standalone unit-test build for the dependency-light estimator cores.
#
# The estimator-core tests intentionally avoid ROS 2 / PCL so they compile with
# nothing more than a C++17 compiler, Eigen and GoogleTest. (The one exception
# is the ikd-Tree concurrency stress test at the bottom, which needs PCL point
# types and is gated on PCL being found.)
#
#     cmake -S resple/test -B build/test && cmake --build build/test
#     ctest --test-dir build/test --output-on-failure
#
# The same sources are also wired into the package build (resple/CMakeLists.txt,
# under BUILD_TESTING) via ament_add_gtest for colcon / CI runs.
cmake_minimum_required(VERSION 3.16)
project(resple_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  # Debian/Ubuntu ship the headers at /usr/include/eigen3 without a config.
  find_path(EIGEN3_INCLUDE_DIR NAMES eigen3/Eigen/Dense Eigen/Dense)
endif()

find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

# Pin Eigen's aligned-allocator dispatch exactly like the package build
# (resple/CMakeLists.txt, HARDENING Phase 1.6): without these, mixed
# preprocessor evaluation across TUs (PCL headers vs plain Eigen) can pair
# aligned_malloc's plain-malloc path with handmade_aligned_free's
# header-word path => heap-buffer-overflow on free. Surfaced here by the
# Phase 5 ASan CI job on test_ikdtree_concurrency.
add_compile_definitions(
  _GNU_SOURCE
  EIGEN_MALLOC_ALREADY_ALIGNED=1
  EIGEN_DEFAULT_ALIGN_BYTES=16)

set(RESPLE_TEST_SOURCES
  test_geometry_core.cpp
  test_point_cloud_adapter.cpp
  test_filter_health.cpp
  test_overload_control.cpp
  test_tf_ownership.cpp
  test_sim_time_wait.cpp
  test_dense_pub.cpp
  test_spline_state.cpp
  test_est_window_anchor.cpp
  test_range_weighting.cpp
)

foreach(src ${RESPLE_TEST_SOURCES})
  get_filename_component(name ${src} NAME_WE)
  # Skip sources that do not exist yet so the series stays bisectable.
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${src})
    add_executable(${name} ${src})
    target_include_directories(${name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../include
      ${EIGEN3_INCLUDE_DIR})
    if(TARGET Eigen3::Eigen)
      target_link_libraries(${name} PRIVATE Eigen3::Eigen)
    endif()
    target_link_libraries(${name} PRIVATE GTest::gtest GTest::gtest_main)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    gtest_discover_tests(${name})
  endif()
endforeach()

# These include SplineState.h, which pulls the estimate_msgs message headers.
# The standalone (ROS-free) build satisfies those includes with the
# field-compatible stubs under test/stubs/; the colcon BUILD_TESTING path
# (resple/CMakeLists.txt) uses the real generated messages instead.
foreach(spline_test test_spline_state test_est_window_anchor)
  if(TARGET ${spline_test})
    target_include_directories(${spline_test} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/stubs)
  endif()
endforeach()

# ikd-Tree concurrency regression (HARDENING Phase 2.4). This one DOES need PCL
# (pcl::PointXYZINormal) and compiles the kd-tree TU directly, so it is gated on
# PCL being available — the cores above still build with only Eigen + GTest.
# Run under ThreadSanitizer with the suppressions file for the strongest signal:
#   cmake -S resple/test -B build/test \
#     -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1 -fno-omit-frame-pointer"
#   cmake --build build/test
#   TSAN_OPTIONS="suppressions=$PWD/resple/test/tsan_suppressions.txt" \
#     ctest --test-dir build/test --output-on-failure -R IkdTreeConcurrency
find_package(PCL QUIET COMPONENTS common)
find_package(Threads REQUIRED)
if(PCL_FOUND AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_ikdtree_concurrency.cpp)
  add_executable(test_ikdtree_concurrency
    test_ikdtree_concurrency.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/ikd-Tree/ikd_Tree.cpp)
  target_include_directories(test_ikdtree_concurrency PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS})
  target_compile_definitions(test_ikdtree_concurrency PRIVATE ${PCL_DEFINITIONS})
  if(TARGET Eigen3::Eigen)
    target_link_libraries(test_ikdtree_concurrency PRIVATE Eigen3::Eigen)
  endif()
  target_link_libraries(test_ikdtree_concurrency PRIVATE
    GTest::gtest GTest::gtest_main Threads::Threads)
  target_compile_options(test_ikdtree_concurrency PRIVATE -Wall -Wextra)
  gtest_discover_tests(test_ikdtree_concurrency PROPERTIES TIMEOUT 300)
else()
  message(STATUS "PCL not found; skipping test_ikdtree_concurrency (ikd-Tree stress).")
endif()
