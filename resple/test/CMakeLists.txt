# Standalone unit-test build for the dependency-light estimator cores.
#
# These tests intentionally avoid ROS 2 / PCL so they compile with nothing more
# than a C++17 compiler, Eigen and GoogleTest:
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

set(RESPLE_TEST_SOURCES
  test_geometry_core.cpp
  test_point_cloud_adapter.cpp
  test_filter_health.cpp
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
