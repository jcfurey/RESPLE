// Unit tests for the adaptive LiDAR outlier gate decision logic
// (utils/lidar_gate.h). Dependency-light: std only, no Eigen, no ROS.
//
// The two properties that matter operationally are pinned here:
//   * ADAPTIVITY  - the same residual is admitted while the covariance is large
//                   and rejected once it has converged. This is the whole reason
//                   the gate is normalized instead of an absolute threshold.
//   * NON-STARVATION - the gate can never remove more than max_reject_frac of an
//                   update's rows, so it cannot lock in a divergence.

#include "utils/lidar_gate.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using resple::gate::MahalanobisGate;

namespace {
// Predicted innovation variance for a converged filter with the shipped
// var_pt = 0.01 (sigma_meas = 0.1 m) and negligible pose uncertainty.
constexpr double kConvergedVar = 0.01;
}  // namespace

TEST(LidarGate, DisabledByDefault) {
  MahalanobisGate g;
  EXPECT_FALSE(g.enabled());
  // Even a wildly inconsistent row passes when the gate is off — the legacy
  // path must be bit-identical.
  EXPECT_FALSE(g.rejects(50.0, kConvergedVar));
  EXPECT_FALSE(g.staysArmed(10, 100));
}

TEST(LidarGate, RejectsBeyondSigmaBoundary) {
  MahalanobisGate g;
  g.sigma = 5.0;
  const double sd = std::sqrt(kConvergedVar);  // 0.1 m
  // Strictly inside / outside the 5-sigma radius (0.5 m here).
  EXPECT_FALSE(g.rejects(4.99 * sd, kConvergedVar));
  EXPECT_FALSE(g.rejects(-4.99 * sd, kConvergedVar));  // sign-symmetric
  EXPECT_TRUE(g.rejects(5.01 * sd, kConvergedVar));
  EXPECT_TRUE(g.rejects(-5.01 * sd, kConvergedVar));
  // Exactly on the boundary is admitted (the test is strict >).
  EXPECT_FALSE(g.rejects(5.0 * sd, kConvergedVar));
}

TEST(LidarGate, WidensWithPoseUncertainty) {
  MahalanobisGate g;
  g.sigma = 5.0;
  // 0.9 m off-plane: an outlier for a converged filter...
  EXPECT_TRUE(g.rejects(0.9, kConvergedVar));
  // ...and entirely legitimate right after init / a gap fast-forward, where
  // H*P*H' contributes ~0.5 m^2 of predicted innovation variance. Same residual,
  // opposite verdict — this is what an absolute nn_thresh cannot express.
  EXPECT_FALSE(g.rejects(0.9, 0.5 + kConvergedVar));
}

TEST(LidarGate, TightensAsFilterConverges) {
  MahalanobisGate g;
  g.sigma = 3.0;
  // 0.35 m: outside 3*sigma_meas (0.3 m) for a fully converged filter, well
  // inside it while H*P*H' still contributes.
  const double residual = 0.35;
  bool rejected_prev = false;
  // Sweep the predicted variance from loose to tight; once the verdict flips to
  // "reject" it must never flip back (monotone in P).
  for (double hph : {1.0, 0.1, 0.01, 1e-3, 1e-4, 0.0}) {
    const bool rejected = g.rejects(residual, hph + kConvergedVar);
    if (rejected_prev) EXPECT_TRUE(rejected) << "hph=" << hph;
    rejected_prev = rejected;
  }
  EXPECT_TRUE(rejected_prev) << "must reject 0.35 m against a converged filter";
}

TEST(LidarGate, NonFiniteResidualRejected) {
  MahalanobisGate g;
  g.sigma = 5.0;
  EXPECT_TRUE(g.rejects(std::numeric_limits<double>::quiet_NaN(), kConvergedVar));
  EXPECT_TRUE(g.rejects(std::numeric_limits<double>::infinity(), kConvergedVar));
  // ...but only when the gate is on; with it off the legacy path decides.
  MahalanobisGate off;
  EXPECT_FALSE(off.rejects(std::numeric_limits<double>::quiet_NaN(), kConvergedVar));
}

TEST(LidarGate, UnjudgeableVarianceAdmits) {
  MahalanobisGate g;
  g.sigma = 5.0;
  // A non-positive or non-finite predicted variance makes the normalized test
  // meaningless. Fall back to admitting rather than rejecting on nonsense.
  EXPECT_FALSE(g.rejects(10.0, 0.0));
  EXPECT_FALSE(g.rejects(10.0, -1.0));
  EXPECT_FALSE(g.rejects(10.0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(g.rejects(10.0, std::numeric_limits<double>::infinity()));
}

TEST(LidarGate, DisarmsWhenRejectingTooMuch) {
  MahalanobisGate g;
  g.sigma = 5.0;
  g.max_reject_frac = 0.5;
  EXPECT_TRUE(g.staysArmed(10, 100));   // 10% — a handful of outliers
  EXPECT_TRUE(g.staysArmed(50, 100));   // exactly at the limit: still armed
  EXPECT_FALSE(g.staysArmed(51, 100));  // past it: the PRIOR is the suspect
  EXPECT_FALSE(g.staysArmed(100, 100)); // total rejection never happens
}

TEST(LidarGate, DisarmEdgeCases) {
  MahalanobisGate g;
  g.sigma = 5.0;
  EXPECT_FALSE(g.staysArmed(0, 100)) << "nothing flagged: nothing to do";
  EXPECT_FALSE(g.staysArmed(5, 0)) << "no candidates: cannot form a fraction";
  EXPECT_FALSE(g.staysArmed(-1, 100));
  g.max_reject_frac = 1.0;
  EXPECT_TRUE(g.staysArmed(100, 100)) << "frac=1 disables the escape hatch";
  g.max_reject_frac = 0.0;
  EXPECT_FALSE(g.staysArmed(1, 1000)) << "frac=0 means never gate";
}

TEST(LidarGate, CannotStarveTheFilter) {
  // Property: for any candidate count and any flagged count, the rows actually
  // dropped are at most max_reject_frac of the candidates. (When the gate
  // disarms, zero rows are dropped.)
  MahalanobisGate g;
  g.sigma = 4.0;
  g.max_reject_frac = 0.3;
  for (int candidates = 1; candidates <= 300; candidates += 7) {
    for (int flagged = 0; flagged <= candidates; ++flagged) {
      const int dropped = g.staysArmed(flagged, candidates) ? flagged : 0;
      EXPECT_LE(static_cast<double>(dropped),
                g.max_reject_frac * static_cast<double>(candidates))
          << "candidates=" << candidates << " flagged=" << flagged;
      EXPECT_GE(candidates - dropped, 1) << "at least one row always survives";
    }
  }
}
