// Integration test (opt-in): CommonUtils::esti_plane over real PCL points.
//
// Validates that the common_utils.h delegation to resple::geom::fitPlane works
// through the actual pcl::PointXYZINormal type and Eigen aligned_vector the
// node uses (the ROS-free unit test exercises the core with Eigen vectors).
//
// Build only with -DRESPLE_BUILD_INTEGRATION_TESTS=ON.

#include "utils/common_utils.h"

#include <gtest/gtest.h>

#include <pcl/point_types.h>

#include <cmath>

namespace {

pcl::PointXYZINormal pt(float x, float y, float z) {
  pcl::PointXYZINormal p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

}  // namespace

TEST(EstiPlanePcl, FitsAxisAlignedPlane) {
  // Plane z = 1 (offset from origin -> well-posed for the A n = -1 form).
  Eigen::aligned_vector<pcl::PointXYZINormal> pts = {
      pt(0, 0, 1), pt(1, 0, 1), pt(0, 1, 1), pt(2, 3, 1), pt(-1, 4, 1)};
  Eigen::Vector4d plane;
  ASSERT_TRUE(CommonUtils::esti_plane<double>(plane, pts, 1e-3));
  EXPECT_NEAR(plane.head<3>().norm(), 1.0, 1e-9);
  EXPECT_NEAR(std::fabs(plane(2)), 1.0, 1e-9);
  for (const auto& p : pts) {
    EXPECT_NEAR(plane(0) * p.x + plane(1) * p.y + plane(2) * p.z + plane(3), 0.0,
                1e-6);
  }
}

TEST(EstiPlanePcl, RejectsNonPlanarPatch) {
  Eigen::aligned_vector<pcl::PointXYZINormal> pts = {
      pt(0, 0, 1), pt(1, 0, 1), pt(0, 1, 1), pt(1, 1, 5)};  // last way off
  Eigen::Vector4d plane;
  EXPECT_FALSE(CommonUtils::esti_plane<double>(plane, pts, 0.05));
}
