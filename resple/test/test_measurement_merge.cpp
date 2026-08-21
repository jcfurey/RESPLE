// Unit tests for the globally chronological per-LiDAR buffer drain used by
// RESPLE::collectMeasurements(). Kept ROS/PCL-free so the starvation
// regression runs in both the standalone and colcon test suites.

#include "utils/measurement_merge.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace {

struct TestPoint {
    int64_t time_ns;
    char sensor;
};

struct TestLidarData {
    std::deque<TestPoint> pt_buff;
};

using TestLidars = std::map<std::string, TestLidarData>;

auto retainAll()
{
    return [](const TestPoint&) { return true; };
}

}  // namespace

TEST(MeasurementMerge, SpendsBudgetOnGloballyEarliestPoints)
{
    // The lexicographically first sensor has enough data to consume the whole
    // budget. A sequential per-sensor drain would therefore starve "z_lidar"
    // despite two of its measurements being globally earlier.
    TestLidars lidars{
        {"a_lidar", {{{100, 'a'}, {300, 'a'}, {400, 'a'}, {500, 'a'}}}},
        {"z_lidar", {{{50, 'z'}, {200, 'z'}}}},
    };
    std::vector<TestPoint> output;

    const std::size_t drained = resple::measurements::drainEarliest(
        lidars, output, 1'000, 3, retainAll());

    ASSERT_EQ(drained, 3u);
    ASSERT_EQ(output.size(), 3u);
    EXPECT_EQ(output[0].time_ns, 50);
    EXPECT_EQ(output[0].sensor, 'z');
    EXPECT_EQ(output[1].time_ns, 100);
    EXPECT_EQ(output[1].sensor, 'a');
    EXPECT_EQ(output[2].time_ns, 200);
    EXPECT_EQ(output[2].sensor, 'z');
    ASSERT_FALSE(lidars.at("a_lidar").pt_buff.empty());
    EXPECT_EQ(lidars.at("a_lidar").pt_buff.front().time_ns, 300);
    EXPECT_TRUE(lidars.at("z_lidar").pt_buff.empty());
}

TEST(MeasurementMerge, StopsAtCommonTimeBoundary)
{
    TestLidars lidars{
        {"a_lidar", {{{10, 'a'}, {30, 'a'}}}},
        {"b_lidar", {{{20, 'b'}, {40, 'b'}}}},
    };
    std::vector<TestPoint> output;

    const std::size_t drained = resple::measurements::drainEarliest(
        lidars, output, 25, 10, retainAll());

    ASSERT_EQ(drained, 2u);
    ASSERT_EQ(output.size(), 2u);
    EXPECT_EQ(output[0].time_ns, 10);
    EXPECT_EQ(output[1].time_ns, 20);
    EXPECT_EQ(lidars.at("a_lidar").pt_buff.front().time_ns, 30);
    EXPECT_EQ(lidars.at("b_lidar").pt_buff.front().time_ns, 40);
}

TEST(MeasurementMerge, DiscardedPointsStillConsumeTheWorkBudget)
{
    // Preserve collectMeasurements' bounded cleanup behavior: measurements
    // outside the retained spline window are popped but still count toward
    // num_points_upd, preventing an unbounded stale-data sweep in one cycle.
    TestLidars lidars{{"lidar", {{{10, 'a'}, {20, 'a'}, {30, 'a'}}}}};
    std::vector<TestPoint> output;

    const std::size_t drained = resple::measurements::drainEarliest(
        lidars, output, 100, 2,
        [](const TestPoint& point) { return point.time_ns >= 30; });

    EXPECT_EQ(drained, 2u);
    EXPECT_TRUE(output.empty());
    ASSERT_EQ(lidars.at("lidar").pt_buff.size(), 1u);
    EXPECT_EQ(lidars.at("lidar").pt_buff.front().time_ns, 30);
}
