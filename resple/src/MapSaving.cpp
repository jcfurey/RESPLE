// MapSaving — standalone PCD accumulator node.
//
// Pulled in from upstream ASIG-X/RESPLE feature/save_map (commit 4a75095,
// "added map saving feature"). Kept verbatim except for this header and the
// exception guards in the two callbacks (an uncaught pcl::IOException from
// savePCDFileBinary — empty cloud / unwritable path — terminated the node).
//
// This is intentionally decoupled from the RESPLE library and from the
// hardened in-node SaveMap *action* (estimate_msgs/action/SaveMap on the
// "save_map" action, which snapshots the live ikd-tree). The two serve
// different workflows:
//   • SaveMap action  -> one-shot snapshot of the current ikd-tree map.
//   • this node        -> accumulates the published `global_map` PointCloud2
//                         *stream* over time, then dumps the union on the
//                         `save_map_node` std_srvs/Empty service.
// Only the upstream MapSaving.cpp + its build target were ported; the
// upstream in-RESPLE.cpp/Mapping.cpp/config changes from that commit were
// deliberately NOT applied (they would duplicate the action and add a second
// map mutex that fights the documented mtx_map_ -> spline_mutex_ ordering).
//
// Run it in the same namespace as the Mapping node (so the relative
// `global_map` topic resolves to the published one), e.g.:
//   ros2 run resple MapSaving --ros-args
//       -r global_map:=/global_map -p pcd_save_path:=/tmp/accumulated_map.pcd
// then trigger a save with:
//   ros2 service call <ns>/save_map_node std_srvs/srv/Empty {}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <mutex>

class MapSaving : public rclcpp::Node
{
public:
    MapSaving() : Node("MapSaving")
    {
        pcd_save_path = this->declare_parameter<std::string>("pcd_save_path", "/tmp/global_map.pcd");

        sub_global_map = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "global_map", 200,
            std::bind(&MapSaving::globalMapCallback, this, std::placeholders::_1));

        srv_save_map = this->create_service<std_srvs::srv::Empty>(
            "save_map_node",
            std::bind(&MapSaving::savePCDCallback, this, std::placeholders::_1, std::placeholders::_2));

        accumulated_map.reset(new pcl::PointCloud<pcl::PointXYZI>());

        RCLCPP_INFO(this->get_logger(), "MapSaving node started, subscribing to 'global_map'.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_global_map;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_save_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map;
    std::mutex mtx_map;
    std::string pcd_save_path;

    void globalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*msg, *cloud);
            std::lock_guard<std::mutex> lock(mtx_map);
            *accumulated_map += *cloud;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "MapSaving: dropped a global_map message on exception: %s", e.what());
        }
    }

    void savePCDCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        pcl::PointCloud<pcl::PointXYZI> map_copy;
        {
            std::lock_guard<std::mutex> lock(mtx_map);
            map_copy = *accumulated_map;
        }
        // savePCDFileBinary throws pcl::IOException (empty cloud, unwritable
        // path); uncaught it escapes the service callback and terminates the
        // node (bug-hunt 2026-07-02 finding #17). std_srvs/Empty has no status
        // field, so log the failure instead.
        if (map_copy.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "Save requested but no map accumulated yet; nothing written to %s",
                pcd_save_path.c_str());
            return;
        }
        try {
            pcl::io::savePCDFileBinary(pcd_save_path, map_copy);
            RCLCPP_INFO(this->get_logger(), "Saved map to %s (%zu points)",
                pcd_save_path.c_str(), map_copy.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map to %s: %s",
                pcd_save_path.c_str(), e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaving>());
    rclcpp::shutdown();
    return 0;
}
