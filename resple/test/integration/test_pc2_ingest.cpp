// Integration test (opt-in): generic PointCloud2 -> pcl::PointXYZINormal.
//
// Unlike the ROS-free unit tests, this exercises the real ROS message type
// (sensor_msgs::msg::PointCloud2) flowing through resple::ingestPointCloud2
// into a PCL cloud, validating the end-to-end ingestion path the node uses.
//
// Build only with -DRESPLE_BUILD_INTEGRATION_TESTS=ON in a ROS 2 workspace; it
// is intentionally excluded from the default / CI test set.

#include "utils/point_cloud_ingest.h"

#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstring>

namespace {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointField makeField(const std::string& name, uint32_t offset, uint8_t dt) {
  PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = dt;
  f.count = 1;
  return f;
}

// Ouster-like cloud: x,y,z float32, intensity float32, t uint32 (ns).
PointCloud2 makeOusterLike(uint32_t n) {
  PointCloud2 msg;
  msg.fields = {makeField("x", 0, PointField::FLOAT32),
                makeField("y", 4, PointField::FLOAT32),
                makeField("z", 8, PointField::FLOAT32),
                makeField("intensity", 12, PointField::FLOAT32),
                makeField("t", 16, PointField::UINT32)};
  msg.point_step = 20;
  msg.height = 1;
  msg.width = n;
  msg.is_bigendian = false;
  msg.row_step = msg.point_step * n;
  msg.data.resize(msg.row_step, 0);
  for (uint32_t i = 0; i < n; ++i) {
    uint8_t* p = msg.data.data() + i * msg.point_step;
    float x = 1.0f + i, y = 0.0f, z = 0.0f, inten = 5.0f * i;
    uint32_t t_ns = i * 100000u;  // 0.1 ms apart
    std::memcpy(p + 0, &x, 4);
    std::memcpy(p + 4, &y, 4);
    std::memcpy(p + 8, &z, 4);
    std::memcpy(p + 12, &inten, 4);
    std::memcpy(p + 16, &t_ns, 4);
  }
  return msg;
}

}  // namespace

TEST(Pc2IngestIntegration, ConvertsOusterLikeCloud) {
  auto msg = makeOusterLike(6);
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  resple::pc2::AdapterConfig cfg;
  ASSERT_TRUE(resple::ingestPointCloud2(msg, /*blind=*/0.0f,
                                        /*point_filter_num=*/1, cfg, cloud));
  ASSERT_EQ(cloud.size(), 6u);
  EXPECT_FLOAT_EQ(cloud.points[0].x, 1.0f);
  // time offset packed into .intensity (ms), reflectivity into .curvature.
  EXPECT_NEAR(cloud.points[1].intensity, 0.1f, 1e-3);
  EXPECT_FLOAT_EQ(cloud.points[2].curvature, 10.0f);
}

TEST(Pc2IngestIntegration, AppliesDecimationAndBlind) {
  auto msg = makeOusterLike(10);
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  resple::pc2::AdapterConfig cfg;
  // blind = 3.5 m drops the first 3 points (x = 1,2,3); 1-in-2 decimation then
  // keeps even indices (0,2,4,6,8) -> remaining x in {5,7,9} after blind.
  ASSERT_TRUE(resple::ingestPointCloud2(msg, /*blind=*/3.5f,
                                        /*point_filter_num=*/2, cfg, cloud));
  for (const auto& pt : cloud.points) {
    EXPECT_GT(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z, 3.5f * 3.5f);
  }
  EXPECT_FALSE(cloud.empty());
}

TEST(Pc2IngestIntegration, RejectsCloudWithoutXyz) {
  PointCloud2 msg;
  msg.fields = {makeField("a", 0, PointField::FLOAT32)};
  msg.point_step = 4;
  msg.height = 1;
  msg.width = 1;
  msg.data.resize(4, 0);
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  resple::pc2::AdapterConfig cfg;
  EXPECT_FALSE(resple::ingestPointCloud2(msg, 0.0f, 1, cfg, cloud));
}
