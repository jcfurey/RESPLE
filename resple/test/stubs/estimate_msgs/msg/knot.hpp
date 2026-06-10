#pragma once
// ROS-free stand-in for the generated estimate_msgs/msg/Knot message, used
// ONLY by the standalone unit-test build (resple/test/CMakeLists.txt adds
// test/stubs/ to the include path; the colcon build uses the real generated
// headers). Field names and meanings mirror estimate_msgs/msg/Knot.msg:
//   geometry_msgs/Point position
//   geometry_msgs/Point orientation_del

namespace estimate_msgs
{
namespace msg
{

struct Knot
{
    struct Point
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    Point position;
    Point orientation_del;
};

}  // namespace msg
}  // namespace estimate_msgs
