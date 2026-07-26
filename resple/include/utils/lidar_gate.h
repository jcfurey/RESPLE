#pragma once

// ---------------------------------------------------------------------------
// lidar_gate.h
//
// Decision logic for the adaptive per-row LiDAR outlier gate used by
// Estimator::updateLiDAR / updateLiDARInertial. std-only (no Eigen, no ROS) so
// it compiles in the dependency-light unit suite — see test/test_lidar_gate.cpp.
//
// WHY THIS EXISTS
//
// The shipped accept test for a point-to-plane row is
//
//     |zp| < nn_thresh   ||   lid_cov < var_pt * coeff_cov
//
// with lid_cov = H·P·Hᵗ + var_pt the predicted innovation variance. Because
// lid_cov >= var_pt by construction, the second disjunct holds for ANY
// converged P as soon as coeff_cov > 1 (production ships 3.0), so it
// short-circuits the first and nn_thresh never binds in steady state. That
// leaves findCorresp's plane-residual test |pd2| < sqrt(range)/9 as the only
// outlier rejection that actually fires — and it admits 0.35 m of off-plane
// error at 10 m, 0.79 m at 50 m, at FULL weight, because the robust kernel is
// off by default. Those are the mid/far-field rows with the longest lever arm
// on the rotation estimate.
//
// Setting coeff_cov = 1.0 makes the second disjunct unsatisfiable and thereby
// activates nn_thresh (this is what config_narrow_tunnel.yaml does). But a
// fixed absolute threshold is the wrong instrument regardless: how much
// residual is admissible depends on how uncertain the pose actually is. Just
// after init, or after a gap fast-forward, metre-scale residuals are entirely
// legitimate; once the filter has converged, decimetre ones are not. A
// normalized (Mahalanobis) test says exactly that:
//
//     reject when  zp² > sigma² · lid_cov
//
// It widens automatically while P is large and tightens as P shrinks, so one
// setting covers both regimes.
//
// THE FAILURE MODE, AND THE ESCAPE HATCH
//
// A normalized gate trusts P. If P is wrong-and-small — an over-confident
// filter whose pose is genuinely off — the gate rejects precisely the
// measurements that would correct it, and the error latches. That is the
// classic gating pathology, and the standard defence is to recognise that a
// LARGE FRACTION of rows failing is evidence against the prior, not against the
// data. So the gate stands down: if more than max_reject_frac of the candidate
// rows would be rejected, it disarms for that update and every row is admitted,
// i.e. exactly the legacy behaviour. The gate can therefore only ever remove a
// minority of any single update's rows — it cannot starve the filter, and it
// cannot lock in a divergence.
//
// Default sigma = 0 disables the whole mechanism (bit-identical legacy path).
// ---------------------------------------------------------------------------

#include <cmath>

namespace resple {
namespace gate {

struct MahalanobisGate {
  // Rejection radius in standard deviations of the PREDICTED innovation, i.e.
  // sqrt(H·P·Hᵗ + var_pt) — not in metres. 0 (default) disables the gate.
  //
  // Scale intuition with the shipped var_pt = 0.01 (σ_meas = 0.1 m): once
  // converged, H·P·Hᵗ is small, so the gate sits near sigma·0.1 m. sigma = 5
  // therefore lands at ~0.5 m — the nn_thresh the configs already specify —
  // but unlike nn_thresh it relaxes on its own while the filter is uncertain.
  double sigma = 0.0;

  // Disarm threshold: if strictly more than this fraction of the update's
  // candidate rows would be rejected, treat that as evidence against the prior
  // and admit everything. 0 disables the escape hatch (not recommended); 1
  // never disarms.
  double max_reject_frac = 0.5;

  bool enabled() const { return sigma > 0.0; }

  // Per-row test. innov_var is the predicted innovation variance (lid_cov).
  //
  // A non-finite residual is rejected: it cannot be admitted meaningfully, and
  // letting it through makes the whole update's LLT fail (which throws away the
  // good rows too). A non-finite or non-positive innov_var makes the test
  // meaningless, so the row is NOT rejected — we simply cannot judge it, and
  // the legacy path is the safe fallback.
  bool rejects(double zp, double innov_var) const {
    if (!enabled()) return false;
    if (!std::isfinite(zp)) return true;
    if (!std::isfinite(innov_var) || !(innov_var > 0.0)) return false;
    return zp * zp > sigma * sigma * innov_var;
  }

  // Does the gate hold after seeing how many rows it wants to drop?
  //
  // `candidates` is the number of rows that passed the legacy accept test (the
  // rows the gate is allowed to judge), `rejected` how many of those it flagged.
  // Returns false — disarm, admit everything — when the flagged fraction
  // exceeds max_reject_frac, and trivially when there is nothing to drop.
  bool staysArmed(int rejected, int candidates) const {
    if (!enabled() || rejected <= 0) return false;
    if (candidates <= 0) return false;
    return static_cast<double>(rejected) <=
           max_reject_frac * static_cast<double>(candidates);
  }
};

}  // namespace gate
}  // namespace resple
