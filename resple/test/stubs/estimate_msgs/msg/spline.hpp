#pragma once
// ROS-free stand-in for the generated estimate_msgs/msg/Spline message, used
// ONLY by the standalone unit-test build (see knot.hpp in this directory).
// Field names and meanings mirror estimate_msgs/msg/Spline.msg:
//   uint64 start_idx
//   int64 dt
//   int64 start_t
//   geometry_msgs/Quaternion start_q
//   Knot[] knots
//   Knot[] idles

#include <cstdint>
#include <vector>

#include "estimate_msgs/msg/knot.hpp"

namespace estimate_msgs
{
namespace msg
{

struct Spline
{
    struct Quaternion
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 1.0;
    };

    uint64_t start_idx = 0;
    int64_t dt = 0;
    int64_t start_t = 0;
    Quaternion start_q;
    std::vector<Knot> knots;
    std::vector<Knot> idles;
};

}  // namespace msg
}  // namespace estimate_msgs
