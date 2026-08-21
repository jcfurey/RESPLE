#pragma once

// Dependency-free chronological merge for per-LiDAR point buffers.
//
// Each individual pt_buff is expected to be timestamp ordered, which is the
// same invariant collectMeasurements historically relied on when draining a
// sensor with a front() loop. Selecting the earliest front on every iteration
// extends that invariant across sensors: the shared point budget is spent on
// the globally oldest eligible measurements instead of map key order.

#include <cstddef>
#include <cstdint>
#include <limits>

namespace resple {
namespace measurements {

template <typename LidarMap, typename OutputBuffer, typename RetainPredicate>
std::size_t drainEarliest(
    LidarMap& lidars,
    OutputBuffer& output,
    int64_t max_time_ns,
    std::size_t max_points,
    RetainPredicate retain)
{
    std::size_t drained = 0;
    while (drained < max_points) {
        auto earliest = lidars.end();
        int64_t earliest_time_ns = std::numeric_limits<int64_t>::max();

        for (auto it = lidars.begin(); it != lidars.end(); ++it) {
            const auto& points = it->second.pt_buff;
            if (points.empty()) {
                continue;
            }
            const int64_t time_ns = points.front().time_ns;
            if (time_ns <= max_time_ns &&
                (earliest == lidars.end() || time_ns < earliest_time_ns)) {
                earliest = it;
                earliest_time_ns = time_ns;
            }
        }

        if (earliest == lidars.end()) {
            break;
        }

        auto& points = earliest->second.pt_buff;
        if (retain(points.front())) {
            output.emplace_back(points.front());
        }
        points.pop_front();
        ++drained;
    }
    return drained;
}

}  // namespace measurements
}  // namespace resple
