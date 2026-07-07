// Unit tests for resple/include/utils/geometry_core.h
//
// These compile and run without ROS 2 / PCL (Eigen + GoogleTest only), so they
// can gate every change to the estimator math in CI and on a developer laptop.

#include "utils/geometry_core.h"

#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

using resple::geom::fitPlane;
using resple::geom::josephUpdate;
using resple::geom::nis;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

namespace {

std::vector<Vec3> onPlaneZ1() {
  // Plane z = 1 (does NOT pass through the origin, so it is well-posed for the
  // A n = -1 normal-equation formulation).
  return {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {2, 3, 1}, {-1, 4, 1}};
}

}  // namespace

// ----------------------------- fitPlane ------------------------------------

TEST(FitPlane, RecoversAxisAlignedPlane) {
  Vec4 out;
  ASSERT_TRUE(fitPlane<double>(onPlaneZ1(), 1e-6, out));
  // Unit normal.
  EXPECT_NEAR(out.head<3>().norm(), 1.0, 1e-9);
  // Normal is ±z.
  EXPECT_NEAR(std::fabs(out(2)), 1.0, 1e-9);
  EXPECT_NEAR(out(0), 0.0, 1e-9);
  EXPECT_NEAR(out(1), 0.0, 1e-9);
  // Every point satisfies n.p + d == 0 (i.e. the plane is z = 1).
  for (const auto& p : onPlaneZ1()) {
    EXPECT_NEAR(out.head<3>().dot(p) + out(3), 0.0, 1e-9);
  }
}

TEST(FitPlane, RecoversTiltedPlane) {
  // Plane: x + 2y + 3z = 6  (offset from origin).
  const Vec3 n_true = Vec3(1, 2, 3).normalized();
  const double d_true = -6.0 / Vec3(1, 2, 3).norm();
  std::vector<Vec3> pts;
  for (auto uv : {std::pair{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {2.0, -1.0},
                  {-3.0, 2.0}}) {
    const double x = uv.first, y = uv.second;
    pts.emplace_back(x, y, (6.0 - x - 2.0 * y) / 3.0);
  }
  Vec4 out;
  ASSERT_TRUE(fitPlane<double>(pts, 1e-6, out));
  // Normal direction matches up to sign.
  EXPECT_NEAR(std::fabs(out.head<3>().dot(n_true)), 1.0, 1e-9);
  // Consistent sign between normal and offset.
  EXPECT_NEAR(out(3) / out.head<3>().dot(n_true), d_true, 1e-9);
}

TEST(FitPlane, RejectsTooFewPoints) {
  Vec4 out;
  std::vector<Vec3> two = {{0, 0, 1}, {1, 0, 1}};
  EXPECT_FALSE(fitPlane<double>(two, 1e-6, out));
}

TEST(FitPlane, CollinearAcceptedByDefaultButRejectedWithConditioning) {
  // Collinear points lie on infinitely many planes; with the default
  // residual-only contract the fit succeeds (this matches the legacy
  // esti_plane behaviour we extracted).
  Vec4 out;
  std::vector<Vec3> line = {{0, 0, 1}, {1, 0, 2}, {2, 0, 3}, {3, 0, 4}};
  EXPECT_TRUE(fitPlane<double>(line, 1e-6, out));
  // Opting into the rank-conditioning guard (HARDENING §3.2) rejects the
  // rank-deficient set, since it does not span a well-defined plane.
  EXPECT_FALSE(fitPlane<double>(line, 1e-6, out, /*min_cond_ratio=*/1e-6));
  // A genuine, well-conditioned plane still passes with the guard enabled.
  EXPECT_TRUE(fitPlane<double>(onPlaneZ1(), 1e-6, out, /*min_cond_ratio=*/1e-6));
}

TEST(FitPlane, RejectsPlaneThroughOrigin) {
  // Documented degenerate case: A n = -1 has no solution when every point lies
  // on a plane through the origin.  Must fail cleanly, not emit garbage.
  Vec4 out;
  std::vector<Vec3> thru = {{1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {2, 3, 0}};
  EXPECT_FALSE(fitPlane<double>(thru, 1e-6, out));
}

TEST(FitPlane, RejectsPointBeyondThreshold) {
  auto pts = onPlaneZ1();
  pts.push_back({5, 5, 1.5});  // 0.5 off the z = 1 plane.
  Vec4 out;
  EXPECT_FALSE(fitPlane<double>(pts, 0.05, out));
  // With a loose threshold the fit (least squares) still succeeds.
  EXPECT_TRUE(fitPlane<double>(pts, 1.0, out));
}

TEST(FitPlane, HandlesNonFiniteInputGracefully) {
  auto pts = onPlaneZ1();
  pts.push_back({std::nan(""), 0, 1});
  Vec4 out;
  EXPECT_FALSE(fitPlane<double>(pts, 1e-6, out));
}

// --------------------------- josephUpdate ----------------------------------

TEST(JosephUpdate, ScalarMatchesClosedForm) {
  Eigen::Matrix<double, 1, 1> P, H, R, K;
  P << 2.0;
  H << 0.5;
  R << 0.3;
  K << 0.8;
  auto Ppost = josephUpdate<1, 1>(P, K, H, R);
  const double kh = (K * H)(0, 0);
  const double expected = (1.0 - kh) * (1.0 - kh) * P(0, 0) + K(0, 0) * K(0, 0) * R(0, 0);
  EXPECT_NEAR(Ppost(0, 0), expected, 1e-12);
}

TEST(JosephUpdate, ResultIsSymmetricAndPSD) {
  // Build an SPD prior and a random gain / Jacobian / noise.
  Eigen::Matrix<double, 4, 4> A = Eigen::Matrix<double, 4, 4>::Random();
  Eigen::Matrix<double, 4, 4> P = A * A.transpose() + 4 * Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Random();
  Eigen::Matrix<double, 2, 2> B = Eigen::Matrix<double, 2, 2>::Random();
  Eigen::Matrix<double, 2, 2> R = B * B.transpose() + Eigen::Matrix2d::Identity();
  // Optimal Kalman gain for this P, H, R.
  Eigen::Matrix<double, 2, 2> S = H * P * H.transpose() + R;
  Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

  auto Ppost = josephUpdate<4, 2>(P, K, H, R);

  // Symmetric.
  EXPECT_LT((Ppost - Ppost.transpose()).norm(), 1e-12);
  // PSD: smallest eigenvalue not meaningfully negative.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(Ppost);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-9);
  // For the optimal gain the posterior must not increase the trace.
  EXPECT_LE(Ppost.trace(), P.trace() + 1e-9);
}

// -------------------------------- nis --------------------------------------

TEST(Nis, IdentityCovarianceGivesSquaredNorm) {
  Eigen::Vector3d v(1, 2, 3);
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
  EXPECT_NEAR(nis(v, S), v.squaredNorm(), 1e-12);
}

TEST(Nis, DiagonalCovarianceScalesComponents) {
  Eigen::Vector3d v(1, 2, 3);
  Eigen::Matrix3d S = Eigen::Vector3d(2, 4, 8).asDiagonal();
  const double expected = 1.0 / 2 + 4.0 / 4 + 9.0 / 8;
  EXPECT_NEAR(nis(v, S), expected, 1e-12);
}

TEST(Nis, NonSpdCovarianceReturnsNaN) {
  Eigen::Vector2d v(1, 1);
  Eigen::Matrix2d S;
  S << 1, 0, 0, -1;  // indefinite -> LLT fails
  EXPECT_TRUE(std::isnan(nis(v, S)));
}

// ---------------------------------------------------------------------------
// subtractBox (HARDENING §3.4 radius map pruning)
// ---------------------------------------------------------------------------

namespace {

using resple::geom::AabBox;
using resple::geom::subtractBox;

double boxVolume(const AabBox& b) {
  return (b.max - b.min).prod();
}

bool contains(const AabBox& b, const Eigen::Vector3d& p) {
  return (p.array() >= b.min.array()).all() && (p.array() < b.max.array()).all();
}

AabBox makeBox(double x0, double y0, double z0, double x1, double y1, double z1) {
  AabBox b;
  b.min = Eigen::Vector3d(x0, y0, z0);
  b.max = Eigen::Vector3d(x1, y1, z1);
  return b;
}

}  // namespace

TEST(SubtractBox, InnerFullyInsideGivesSixSlabsTilingTheComplement) {
  const AabBox outer = makeBox(-10, -10, -10, 10, 10, 10);
  const AabBox inner = makeBox(-2, -3, -4, 5, 4, 3);
  const auto slabs = subtractBox(outer, inner);
  ASSERT_EQ(slabs.size(), 6u);

  double vol = 0;
  for (const auto& s : slabs) vol += boxVolume(s);
  EXPECT_NEAR(vol, boxVolume(outer) - boxVolume(inner), 1e-9);

  // Deterministic dense sampling: every sample inside `outer` must be in the
  // inner box XOR exactly one slab (disjoint + covering).
  for (double x = -9.5; x < 10; x += 1.0)
    for (double y = -9.5; y < 10; y += 1.0)
      for (double z = -9.5; z < 10; z += 1.0) {
        const Eigen::Vector3d p(x, y, z);
        int hits = 0;
        for (const auto& s : slabs) hits += contains(s, p);
        if (contains(inner, p)) {
          EXPECT_EQ(hits, 0) << p.transpose();
        } else {
          EXPECT_EQ(hits, 1) << p.transpose();
        }
      }
}

TEST(SubtractBox, InnerCoveringOuterGivesNothing) {
  const AabBox outer = makeBox(-1, -1, -1, 1, 1, 1);
  const AabBox inner = makeBox(-2, -2, -2, 2, 2, 2);
  EXPECT_TRUE(subtractBox(outer, inner).empty());
  // Exactly equal boxes likewise leave nothing.
  EXPECT_TRUE(subtractBox(outer, outer).empty());
}

TEST(SubtractBox, DisjointInnerReturnsOuterExactly) {
  const AabBox outer = makeBox(0, 0, 0, 4, 4, 4);
  const AabBox inner = makeBox(10, 10, 10, 12, 12, 12);
  const auto slabs = subtractBox(outer, inner);
  double vol = 0;
  for (const auto& s : slabs) vol += boxVolume(s);
  EXPECT_NEAR(vol, boxVolume(outer), 1e-9);
}

TEST(SubtractBox, PartialOverlapTilesTheDifference) {
  const AabBox outer = makeBox(0, 0, 0, 10, 10, 10);
  // Sticks out of `outer` on +x and -y; overlaps the rest.
  const AabBox inner = makeBox(6, -5, 2, 15, 7, 9);
  const auto slabs = subtractBox(outer, inner);
  for (double x = 0.5; x < 10; x += 1.0)
    for (double y = 0.5; y < 10; y += 1.0)
      for (double z = 0.5; z < 10; z += 1.0) {
        const Eigen::Vector3d p(x, y, z);
        int hits = 0;
        for (const auto& s : slabs) hits += contains(s, p);
        EXPECT_EQ(hits, contains(inner, p) ? 0 : 1) << p.transpose();
      }
}

// ------------------------- attitude / gravity init -------------------------
// These lock the invariant the TF-attitude init (Design B) rests on: for a
// gravity-aligned reference frame, the attitude derived from the TF (base_link
// pose in that frame, yaw removed) equals the attitude derived from the
// accelerometer (measured gravity direction), to within noise. If these two
// constructions ever diverge, the cross-check in initialization() would fire
// on correct data.

using resple::geom::g2r;
using resple::geom::r2ypr;
using resple::geom::ypr2r;

namespace {
// Specific force an accelerometer at rest reads, expressed in body frame:
// world gravity is [0,0,+g] (up); the sensor measures it rotated into body.
Vec3 measuredGravityBody(const Eigen::Matrix3d& R_wb, double g = 9.81) {
  return R_wb.transpose() * Vec3(0, 0, g);
}
}  // namespace

TEST(Attitude, YprRoundTrip) {
  for (double yaw : {-170.0, -40.0, 0.0, 33.0, 175.0})
    for (double pitch : {-70.0, -15.0, 0.0, 25.0, 60.0})
      for (double roll : {-75.0, -10.0, 0.0, 20.0, 80.0}) {
        const Vec3 ypr(yaw, pitch, roll);
        const Vec3 back = r2ypr(ypr2r(ypr));
        EXPECT_NEAR(back.x(), yaw, 1e-9) << ypr.transpose();
        EXPECT_NEAR(back.y(), pitch, 1e-9) << ypr.transpose();
        EXPECT_NEAR(back.z(), roll, 1e-9) << ypr.transpose();
      }
}

TEST(Attitude, GravityRecoversRollPitchZeroesYaw) {
  // g2r(measured gravity) must recover the platform roll/pitch and zero yaw,
  // independent of the true yaw (gravity cannot observe heading).
  for (double yaw : {-120.0, 0.0, 95.0})
    for (double pitch : {-60.0, -20.0, 0.0, 30.0, 65.0})
      for (double roll : {-70.0, 0.0, 15.0, 55.0}) {
        const Eigen::Matrix3d R_wb = ypr2r(Vec3(yaw, pitch, roll));
        const Eigen::Matrix3d R_grav = g2r(measuredGravityBody(R_wb));
        const Vec3 ypr = r2ypr(R_grav);
        EXPECT_NEAR(ypr.x(), 0.0, 1e-6) << "yaw must be zeroed";
        EXPECT_NEAR(ypr.y(), pitch, 1e-6) << "pitch, true yaw=" << yaw;
        EXPECT_NEAR(ypr.z(), roll, 1e-6) << "roll, true yaw=" << yaw;
      }
}

TEST(Attitude, TfAndGravityAgreeInLevelFrame) {
  // The Design B cross-check: with a gravity-aligned attitude frame, the
  // TF-derived q_WI (base_link pose, yaw removed) matches the accelerometer
  // q_WI. Delta (roll/pitch, degrees) must be ~0 on consistent data.
  for (double yaw : {-150.0, 0.0, 60.0})
    for (double pitch : {-55.0, 0.0, 40.0})
      for (double roll : {-65.0, 0.0, 25.0}) {
        const Eigen::Matrix3d R_wb = ypr2r(Vec3(yaw, pitch, roll));  // TF: base_link in world
        // TF path: take roll/pitch from the TF, zero the yaw gauge.
        Vec3 tf_ypr = r2ypr(R_wb);
        tf_ypr.x() = 0.0;
        const Vec3 q_tf = r2ypr(ypr2r(tf_ypr));
        // Gravity path.
        const Vec3 q_grav = r2ypr(g2r(measuredGravityBody(R_wb)));
        const double dpitch = q_tf.y() - q_grav.y();
        const double droll = q_tf.z() - q_grav.z();
        EXPECT_LT(std::sqrt(dpitch * dpitch + droll * droll), 1e-4)
            << "ypr=(" << yaw << "," << pitch << "," << roll << ")";
      }
}

TEST(Attitude, MountingTiltShowsAsCrossCheckDelta) {
  // A wrong imu->base_link extrinsic (the accel is rotated by an unmodeled
  // mounting tilt) must surface as a nonzero TF-vs-gravity delta — the
  // diagnostic value that makes the cross-check useful.
  const Eigen::Matrix3d R_wb = ypr2r(Vec3(10.0, 5.0, -8.0));  // true, level-ish base_link
  const Eigen::Matrix3d R_mount = ypr2r(Vec3(0.0, 12.0, 0.0));  // unmodeled 12deg pitch mount
  // Gravity measured in the (mis-mounted, uncorrected) IMU frame.
  const Vec3 g_imu = R_mount * measuredGravityBody(R_wb);
  const Vec3 q_grav = r2ypr(g2r(g_imu));
  Vec3 tf_ypr = r2ypr(R_wb);
  tf_ypr.x() = 0.0;
  const Vec3 q_tf = r2ypr(ypr2r(tf_ypr));
  const double dpitch = q_tf.y() - q_grav.y();
  const double droll = q_tf.z() - q_grav.z();
  const double delta = std::sqrt(dpitch * dpitch + droll * droll);
  EXPECT_GT(delta, 5.0) << "12deg mount tilt should be visible in the delta";
}
