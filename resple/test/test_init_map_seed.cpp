// Regression tests for rotating-head initial-map boundary selection.  Kept
// ROS/PCL-free so the same cases run in standalone and colcon test suites.

#include "utils/init_map_seed.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <optional>

namespace {

struct TimedPoint {
    int64_t time_ns;
    double offset_ms;
};

std::optional<std::size_t> seedEnd(
    const std::deque<TimedPoint>& points, int64_t requested_end_ns)
{
    return resple::initialization::completeScanSeedEnd(
        points,
        requested_end_ns,
        [](const TimedPoint& point) { return point.time_ns; },
        [](const TimedPoint& point) { return point.offset_ms; });
}

}  // namespace

TEST(InitMapSeed, WaitsUntilRequestedTimeAndFollowingScanBoundaryExist)
{
    EXPECT_FALSE(seedEnd({}, 15).has_value());
    EXPECT_FALSE(seedEnd({{0, 0.0}, {10, 10.0}}, 15).has_value());
    EXPECT_FALSE(
        seedEnd({{0, 0.0}, {10, 10.0}, {20, 20.0}}, 15).has_value());
}

TEST(InitMapSeed, SnapsMidScanCutoffToNextScan)
{
    const std::deque<TimedPoint> points{
        {0, 0.0}, {10, 10.0}, {20, 20.0},
        {30, 0.0}, {40, 10.0}, {50, 20.0},
    };

    const auto end = seedEnd(points, 15);

    ASSERT_TRUE(end.has_value());
    EXPECT_EQ(*end, 3u);
}

TEST(InitMapSeed, KeepsExactBoundaryForFirstPostSeedScan)
{
    const std::deque<TimedPoint> points{
        {0, 0.0}, {10, 10.0}, {20, 20.0},
        {30, 0.0}, {40, 10.0}, {50, 20.0},
    };

    const auto end = seedEnd(points, 30);

    ASSERT_TRUE(end.has_value());
    EXPECT_EQ(*end, 3u);
}

TEST(InitMapSeed, IgnoresSmallOffsetJitter)
{
    const std::deque<TimedPoint> points{
        {0, 0.0}, {10, 1.0}, {20, 0.9995}, {30, 2.0}, {40, 0.0},
    };

    const auto end = seedEnd(points, 15);

    ASSERT_TRUE(end.has_value());
    EXPECT_EQ(*end, 4u);
}
