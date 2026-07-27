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
  // Accumulate and solve in DOUBLE regardless of T. The float instantiation
  // (esti_plane's Vector4f path) previously formed A^T A in float32: the
  // normal equations square the condition number, and for world-frame patches
  // far from the origin (coords ~1e2-1e3 m, patch extent ~0.1 m) the
  // informative variation fell below float's ~7 significant digits, degrading
  // the fitted normal exactly where the map is sparsest. Double accumulation
  // keeps 16 digits through the squaring; the rank guard and the residual
  // gate below still reject genuinely degenerate patches.
  Eigen::Matrix3d ATA = Eigen::Matrix3d::Zero();
  Eigen::Vector3d ATb = Eigen::Vector3d::Zero();
  for (int j = 0; j < num_pts; ++j) {
    T x, y, z;
    get_xyz(j, x, y, z);
    Eigen::Vector3d row(static_cast<double>(x), static_cast<double>(y),
                        static_cast<double>(z));
    ATA.noalias() += row * row.transpose();
    ATb.noalias() -= row;
  }
  Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr(ATA);
  if (min_cond_ratio > static_cast<T>(0)) {
    // Degeneracy test on the CENTERED scatter S = Σ (p−c)(p−c)ᵀ.
    //
    // This used to be a rank test on ATA = Σ p pᵀ directly. That matrix is
    // dominated by the centroid outer product k·c·cᵀ, so its pivot ratio is a
    // function of where the patch sits relative to the ODOM ORIGIN, not of
    // whether the neighbours span a plane: measured, it falls about five
    // decades from 1 m to 200 m out, and by 20 m the good-patch and
    // collinear-patch distributions are identical to three digits. A single
    // global threshold therefore had no stable meaning — tuned near the start
    // pose it rejected nothing; carried 20 m out it rejected almost
    // everything. Centering removes the c·cᵀ term and makes the test
    // translation-invariant, which is what the fit's conditioning actually
    // depends on.
    //
    // The test also has to INVERT. A good planar patch is rank 2 in S (the
    // off-plane eigenvalue is pure noise), so "rank(S) == 3" would reject
    // exactly the patches we want. The discriminator between a plane and a
    // line is the ratio of the two IN-PLANE extents: λ2/λ1 near 1 is a
    // well-spread patch, λ2/λ1 → 0 is collinear.
    //
    // Note this changes the parameter's numeric scale. It is now a normalized
    // eigenvalue ratio in (0,1] — 0.05 means the second in-plane extent is at
    // least ~22% of the first — where before it was a pivot ratio of ATA whose
    // usable values were ~1e-6 and distance-dependent. See doc/PARAMETERS.md.
    const Eigen::Vector3d centroid = -ATb / static_cast<double>(num_pts);
    Eigen::Matrix3d S = ATA;
    S.noalias() -= static_cast<double>(num_pts) * centroid * centroid.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(S);
    if (es.info() != Eigen::Success) {
      return false;
    }
    // Ascending order: (0) off-plane, (1) minor in-plane, (2) major in-plane.
    const double lambda_major = es.eigenvalues()(2);
    const double lambda_minor = es.eigenvalues()(1);
    if (!(lambda_major > 0.0) ||
        lambda_minor / lambda_major < static_cast<double>(min_cond_ratio)) {
      return false;
    }
  }
  const Eigen::Vector3d normvec = qr.solve(ATb);
  if (!normvec.allFinite()) {
    return false;
  }
  const double n = normvec.norm();
  if (n < 1e-12) {
    return false;
  }
  const double n_inv = 1.0 / n;
  out(0) = static_cast<T>(normvec(0) * n_inv);
  out(1) = static_cast<T>(normvec(1) * n_inv);
  out(2) = static_cast<T>(normvec(2) * n_inv);
  out(3) = static_cast<T>(n_inv);
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

// ---------------------------------------------------------------------------
// Attitude helpers (yaw-pitch-roll <-> rotation, gravity alignment).
//
// Pure-Eigen so the gravity-init math is unit-testable without ROS.
// CommonUtils::{R2ypr,ypr2R,g2R} forward here — one definition. Angle
// convention is DEGREES (historical, matching the node) and the rotation
// order is R = Rz(yaw)·Ry(pitch)·Rx(roll). Used by the initial-attitude
// estimation (gravity alignment and the TF-attitude path).
// ---------------------------------------------------------------------------
inline Eigen::Vector3d r2ypr(const Eigen::Matrix3d& R) {
  const Eigen::Vector3d n = R.col(0);
  const Eigen::Vector3d o = R.col(1);
  const Eigen::Vector3d a = R.col(2);
  const double y = std::atan2(n(1), n(0));
  const double p = std::atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
  const double r = std::atan2(a(0) * std::sin(y) - a(1) * std::cos(y),
                              -o(0) * std::sin(y) + o(1) * std::cos(y));
  return Eigen::Vector3d(y, p, r) / M_PI * 180.0;
}

inline Eigen::Matrix3d ypr2r(const Eigen::Vector3d& ypr) {
  const double y = ypr(0) / 180.0 * M_PI;
  const double p = ypr(1) / 180.0 * M_PI;
  const double r = ypr(2) / 180.0 * M_PI;
  Eigen::Matrix3d Rz;
  Rz << std::cos(y), -std::sin(y), 0,
        std::sin(y),  std::cos(y), 0,
        0,            0,           1;
  Eigen::Matrix3d Ry;
  Ry <<  std::cos(p), 0, std::sin(p),
         0,           1, 0,
        -std::sin(p), 0, std::cos(p);
  Eigen::Matrix3d Rx;
  Rx << 1, 0,           0,
        0, std::cos(r), -std::sin(r),
        0, std::sin(r),  std::cos(r);
  return Rz * Ry * Rx;
}

// Rotation taking the measured gravity direction to world +Z, with yaw
// removed — gravity observes only roll/pitch, never yaw, so the yaw gauge is
// left free (zeroed). NOTE: FromTwoVectors is ill-conditioned as `g`
// approaches anti-parallel to +Z (platform inverted); callers on
// steeply-tilted rigs should prefer the TF-attitude path.
inline Eigen::Matrix3d g2r(const Eigen::Vector3d& g) {
  const Eigen::Vector3d ng1 = g.normalized();
  const Eigen::Vector3d ng2{0, 0, 1.0};
  Eigen::Matrix3d R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
  const double yaw = r2ypr(R0).x();
  R0 = ypr2r(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  return R0;
}

// ---------------------------------------------------------------------------
// Quaternion-perturbation -> 3-vector maps for pose covariance reporting.
//
// A 4D quaternion covariance is turned into the 3x3 rotation block ROS wants by
// P_rot = (G J_q) P (G J_q)^T, where G maps a quaternion perturbation dq to a
// 3-vector of small rotations. WHICH 3-vector depends on the side the
// perturbation is applied:
//
//   BODY  (right/local) : dphi_b = 2*imag(q^-1 (x) dq) = 2*rows1..3 Qleft(q^-1)
//   WORLD (fixed-axis)  : dphi_w = 2*imag(dq (x) q^-1) = 2*rows1..3 Qright(q^-1)
//
// and the two are related by dphi_w = R(q) * dphi_b, so
// P_world = R P_body R^T.
//
// `geometry_msgs/PoseWithCovariance` specifies "the orientation parameters use a
// fixed-axis representation", i.e. rotations about the axes of the message's
// header.frame_id -- so a pose published in `odom` must report the WORLD form.
// robot_localization rotates an incoming pose covariance only from the message
// frame into its world frame; with frame_id == odom that rotation is identity,
// so a body-frame block is consumed verbatim as world roll/pitch/yaw and any
// anisotropy lands on the wrong axes (it over-trusts the least-observable one).
//
// Both forms are provided: the world one is what publishers must use, and the
// body one exists so the relation above stays unit-testable.
// ---------------------------------------------------------------------------
inline Eigen::Matrix<double, 3, 4> quatPerturbToBody(const Eigen::Quaterniond& q)
{
  const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  Eigen::Matrix<double, 3, 4> G;
  G << -qx,  qw,  qz, -qy,
       -qy, -qz,  qw,  qx,
       -qz,  qy, -qx,  qw;
  return 2.0 * G;
}

inline Eigen::Matrix<double, 3, 4> quatPerturbToWorld(const Eigen::Quaterniond& q)
{
  const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  Eigen::Matrix<double, 3, 4> G;
  G << -qx,  qw, -qz,  qy,
       -qy,  qz,  qw, -qx,
       -qz, -qy,  qx,  qw;
  return 2.0 * G;
}

// ---------------------------------------------------------------------------
// Axis-aligned box subtraction (HARDENING §3.4 radius map pruning).
//
// Carves `outer \ inner` into at most 6 axis-aligned slabs. The returned
// boxes are clipped to `outer`, have positive extent on every axis, are
// pairwise disjoint, and together with `outer ∩ inner` exactly tile `outer`
// (boundaries are treated half-open, consistent with the ikd-Tree's
// box-deletion semantics being applied to disjoint regions). Used to delete
// every map point outside a retention box without touching the points
// inside it.
// ---------------------------------------------------------------------------
struct AabBox {
  Eigen::Vector3d min;
  Eigen::Vector3d max;
};

inline std::vector<AabBox> subtractBox(const AabBox& outer,
                                       const AabBox& inner) {
  std::vector<AabBox> out;
  AabBox rem = outer;  // remaining (not yet carved) region
  for (int i = 0; i < 3; ++i) {
    if (inner.min(i) > rem.min(i)) {
      AabBox b = rem;
      b.max(i) = std::min(inner.min(i), rem.max(i));
      if ((b.max.array() > b.min.array()).all()) {
        out.push_back(b);
      }
      rem.min(i) = b.max(i);
    }
    if (inner.max(i) < rem.max(i)) {
      AabBox b = rem;
      b.min(i) = std::max(inner.max(i), rem.min(i));
      if ((b.max.array() > b.min.array()).all()) {
        out.push_back(b);
      }
      rem.max(i) = b.min(i);
    }
    if ((rem.max.array() <= rem.min.array()).any()) {
      break;  // nothing left of `outer` to carve
    }
  }
  return out;
}

}  // namespace geom
}  // namespace resple
