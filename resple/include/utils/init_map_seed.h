#pragma once

// Dependency-free initial-map boundary selection.
//
// A long seed may end partway through a LiDAR scan.  Returning the index of
// the first point in the following scan lets initialization consume only
// complete scans and leaves a complete scan for the first estimator update.

#include <cstddef>
#include <cstdint>
#include <optional>

namespace resple {
namespace initialization {

template <typename PointBuffer, typename TimeAccessor, typename OffsetAccessor>
std::optional<std::size_t> completeScanSeedEnd(
    const PointBuffer& points,
    int64_t requested_end_ns,
    TimeAccessor time_ns,
    OffsetAccessor offset_ms,
    double reset_epsilon_ms = 1.0e-3)
{
    if (points.empty() || time_ns(points.back()) < requested_end_ns) {
        return std::nullopt;
    }

    std::size_t boundary = 0;
    while (boundary < points.size() &&
           time_ns(points[boundary]) < requested_end_ns) {
        ++boundary;
    }

    // Points are ordered by their offset within each scan.  The offset rises
    // through a scan and drops at the first point of the next one.  Ignore
    // sub-epsilon numeric jitter so it cannot manufacture a false boundary.
    while (boundary < points.size()) {
        if (boundary > 0 &&
            static_cast<double>(offset_ms(points[boundary])) +
                    reset_epsilon_ms <
                static_cast<double>(offset_ms(points[boundary - 1]))) {
            return boundary;
        }
        ++boundary;
    }
    return std::nullopt;
}

}  // namespace initialization
}  // namespace resple
