#pragma once

// ---------------------------------------------------------------------------
// geometry_core.h
//
// Dependency-light (Eigen-only) numeric cores for the estimator.  These are
// deliberately free of ROS 2, PCL and Boost so they can be compiled and
// unit-tested on their own (see resple/test/).  The heavy translation units
// (common_utils.h, Estimator.h) delegate to these so the math has exactly one
// source of truth and that source is covered by tests.
//
//   * fitPlane      - least-squares plane fit used for LiDAR correspondences
//   * josephUpdate  - PSD-preserving IEKF posterior covariance
//   * nis           - normalized innovation squared (filter-health / gating)
//
// All functions are header-only and templated/inline; codegen is identical to
// hand-inlined code, so there is no hot-path cost to routing through them.
// ---------------------------------------------------------------------------

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <vector>

namespace resple {
namespace geom {

// ---------------------------------------------------------------------------
// Plane fit.
//
// Solves A n = b with rows A_j = [x_j, y_j, z_j] and b_j = -1 in the
// normal-equation form (A^T A) n = A^T b, using a rank-revealing
// colPivHouseholderQr so degenerate inputs (collinear points, a patch passing
// through the origin) are rejected instead of producing garbage.
//
// On success `out` holds the Hessian-normal plane [nx, ny, nz, d] with a unit
// normal (so |out.head<3>()| == 1) and every input point satisfies
// |n . p + d| <= threshold.  Returns false when:
//   * fewer than 3 points are supplied,
//   * the normal-equation solve is non-finite or near-zero (degenerate), or
//   * any point lies farther than `threshold` from the fitted plane.
//
// `get_xyz(j, x, y, z)` writes the j-th point's coordinates.  Templating on the
// accessor lets callers feed PCL points, Eigen vectors or raw arrays with no
// intermediate copy.
//
// `min_cond_ratio` is an optional rank-conditioning guard (HARDENING §3.2): a
// collinear or otherwise rank-deficient point set lies on infinitely many
// planes, so a residual-only fit happily accepts it and returns an arbitrary
// (unreliable) normal.  When `min_cond_ratio > 0` the rank-revealing QR
// rejects such patches.  It defaults to 0 (disabled) so the fit is bit-for-bit
// equivalent to the legacy residual-only behaviour unless a caller opts in.
// ---------------------------------------------------------------------------
template <typename T, typename Accessor>
inline bool fitPlane(int num_pts, Accessor&& get_xyz, T threshold,
                     Eigen::Matrix<T, 4, 1>& out,
                     T min_cond_ratio = static_cast<T>(0)) {
  if (num_pts < 3) {
    return false;
  }
  Eigen::Matrix<T, 3, 3> ATA = Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, 3, 1> ATb = Eigen::Matrix<T, 3, 1>::Zero();
  for (int j = 0; j < num_pts; ++j) {
    T x, y, z;
    get_xyz(j, x, y, z);
    Eigen::Matrix<T, 3, 1> row(x, y, z);
    ATA.noalias() += row * row.transpose();
    ATb.noalias() -= row;
  }
  Eigen::ColPivHouseholderQR<Eigen::Matrix<T, 3, 3>> qr(ATA);
  if (min_cond_ratio > static_cast<T>(0)) {
    // Threshold is relative to the largest pivot, so the rank test is
    // scale-invariant.  rank < 3 ⇒ the points do not span a well-defined plane.
    qr.setThreshold(min_cond_ratio);
    if (qr.rank() < 3) {
      return false;
    }
  }
  Eigen::Matrix<T, 3, 1> normvec = qr.solve(ATb);
  if (!normvec.allFinite()) {
    return false;
  }
  const T n = normvec.norm();
  if (n < static_cast<T>(1e-12)) {
    return false;
  }
  const T n_inv = static_cast<T>(1.0) / n;
  out(0) = normvec(0) * n_inv;
  out(1) = normvec(1) * n_inv;
  out(2) = normvec(2) * n_inv;
  out(3) = n_inv;
  for (int j = 0; j < num_pts; ++j) {
    T x, y, z;
    get_xyz(j, x, y, z);
    if (std::fabs(out(0) * x + out(1) * y + out(2) * z + out(3)) > threshold) {
      return false;
    }
  }
  return true;
}

// Convenience overload over a contiguous span of Eigen 3-vectors.
template <typename T>
inline bool fitPlane(const std::vector<Eigen::Matrix<T, 3, 1>>& pts,
                     T threshold, Eigen::Matrix<T, 4, 1>& out,
                     T min_cond_ratio = static_cast<T>(0)) {
  return fitPlane(static_cast<int>(pts.size()),
                  [&pts](int j, T& x, T& y, T& z) {
                    x = pts[j](0);
                    y = pts[j](1);
                    z = pts[j](2);
                  },
                  threshold, out, min_cond_ratio);
}

// ---------------------------------------------------------------------------
// Joseph-form posterior covariance.
//
//     P_post = (I - K H) P (I - K H)^T + K R K^T
//
// This form preserves positive-semi-definiteness in finite precision, unlike
// the algebraically-equivalent (I - K H) P.  The result is symmetrized to
// scrub the last bit of round-off asymmetry before it feeds the next
// propagation.  Templated on the state size N and measurement size M so the
// fixed-size estimator matrices compile to the same code as a hand inline.
// ---------------------------------------------------------------------------
template <int N, int M, typename Scalar = double>
inline Eigen::Matrix<Scalar, N, N> josephUpdate(
    const Eigen::Matrix<Scalar, N, N>& P,
    const Eigen::Matrix<Scalar, N, M>& K,
    const Eigen::Matrix<Scalar, M, N>& H,
    const Eigen::Matrix<Scalar, M, M>& R) {
  const Eigen::Matrix<Scalar, N, N> I_KH =
      Eigen::Matrix<Scalar, N, N>::Identity() - K * H;
  Eigen::Matrix<Scalar, N, N> P_post = I_KH * P * I_KH.transpose();
  P_post.noalias() += K * R * K.transpose();
  return static_cast<Scalar>(0.5) * (P_post + P_post.transpose());
}

// ---------------------------------------------------------------------------
// Normalized innovation squared (NIS):  nu^T S^{-1} nu.
//
// With a consistent filter NIS follows a chi-squared distribution with M
// (= measurement-dimension) degrees of freedom, so a windowed NIS that drifts
// far above M signals divergence / over-confident covariance.  Returns a
// non-finite value (propagated NaN) when S is not solvable so callers can gate
// on allFinite() rather than trusting a bogus scalar.
// ---------------------------------------------------------------------------
template <typename DerivedV, typename DerivedS>
inline typename DerivedV::Scalar nis(const Eigen::MatrixBase<DerivedV>& innov,
                                     const Eigen::MatrixBase<DerivedS>& S) {
  using Scalar = typename DerivedV::Scalar;
  Eigen::LLT<typename DerivedS::PlainObject> llt(S);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<Scalar>::quiet_NaN();
  }
  // x = S^{-1} nu  via the Cholesky factor (no explicit inverse).
  const typename DerivedV::PlainObject x = llt.solve(innov);
  return innov.dot(x);
}

}  // namespace geom
}  // namespace resple
