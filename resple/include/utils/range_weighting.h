#pragma once

// ---------------------------------------------------------------------------
// range_weighting.h
//
// Dependency-light (std-only) core for the LiDAR close-range measurement
// down-weighting. Modeled on utils/filter_health.h / overload_control.h: pure
// functions, no ROS/PCL/Eigen, unit-tested in resple/test/test_range_weighting.cpp.
//
// WHY THE DOWN-WEIGHTING EXISTS (commit 3bffada): a point-to-plane residual's
// information enters the IEKF as 1/(var_pt * scale). Returns very close to the
// sensor are both geometrically over-represented (solid angle -> many returns
// per surface) and disproportionately leveraged on rotation, so a single nearby
// surface in an otherwise-open scene can dominate H^T R^-1 H and drag the pose.
// Inflating their variance by (range_ref/r)^2 below range_ref suppresses that.
//
// WHY THE ABSOLUTE FORM IS A TRAP: the factor depends only on the point's own
// range, so it is blind to the rest of the scene. In a confined space -- a
// narrow tunnel, a culvert, the back of a truck -- EVERY return is close, so
// every row is inflated by the same 9x (walls at 1 m) or 36x (0.5 m). Uniform
// inflation does not re-balance anything; it just scales the whole LiDAR
// information block down relative to the (unchanged) prior P^-1, so the filter
// coasts on the prior/IMU and then snaps when geometry returns. That is
// information starvation, not outlier protection.
//
// THE RELATIVE FORM: normalize each scan's scales by the scan's own minimum,
// i.e. by the scale of its FARTHEST valid return. `rangeNoiseScale` is
// non-increasing in range, so that minimum is exactly scale(max_range).
//
//   * open scene, a few close returns -> the far returns already sit at
//     scale 1, so the normalizer is 1 and behaviour is UNCHANGED (the
//     anti-domination property is preserved exactly);
//   * uniformly-close scene -> all scales are equal, so all normalize to 1:
//     no starvation, and the relative weighting between returns (which is what
//     the mechanism is actually for) is identical;
//   * mixed scene -> relative weighting preserved, absolute level anchored to
//     the best-conditioned return in the scan.
//
// Cost is one min-reduction over the update's points (<= num_points_upd, a few
// hundred floats, no allocation), so it is free on the hot path.
// ---------------------------------------------------------------------------

#include <algorithm>

namespace resple {
namespace range {

struct RangeNoiseConfig {
  // Below this range (m) a return's variance is inflated. <= 0 disables the
  // whole mechanism (scale == 1 everywhere).
  double range_ref = 3.0;
  // Ceiling on the inflation factor. The default reproduces the historical
  // implicit (3 / 0.1)^2 ceiling that came from the 0.1 m range floor below.
  double scale_max = 900.0;
  // false = absolute (legacy, bit-identical); true = per-scan relative, i.e.
  // normalized by the scan's farthest valid return. See the header comment.
  bool relative = false;
};

// Raw (absolute) variance inflation factor for one return. Non-increasing in
// `range`, clamped at a 0.1 m range floor and at cfg.scale_max.
inline double rangeNoiseScale(const RangeNoiseConfig& cfg, float range)
{
  if (cfg.range_ref <= 0.0) {
    return 1.0;
  }
  const double r = std::max(static_cast<double>(range), 0.1);
  if (r >= cfg.range_ref) {
    return 1.0;
  }
  const double scale = (cfg.range_ref * cfg.range_ref) / (r * r);
  return std::min(scale, cfg.scale_max);
}

// Per-update normalizer. `max_valid_range` is the largest range among the
// returns actually being fed to this update (validity matters: rejected rows
// carry no information, so they must not set the reference). Returns 1.0 when
// the feature is off, when the mechanism is disabled, or when no valid return
// was supplied -- so the caller can divide unconditionally.
inline double rangeNoiseNormalizer(const RangeNoiseConfig& cfg,
                                   float max_valid_range)
{
  if (!cfg.relative || cfg.range_ref <= 0.0 || !(max_valid_range > 0.0f)) {
    return 1.0;
  }
  return rangeNoiseScale(cfg, max_valid_range);
}

// The factor the IEKF should divide a row's information by:
//   R_inv = robust_weight / (var_pt * rangeNoiseWeight(...))
// Equivalent to rangeNoiseScale() in absolute mode.
inline double rangeNoiseWeight(const RangeNoiseConfig& cfg, float range,
                               double normalizer)
{
  const double w = rangeNoiseScale(cfg, range);
  return (normalizer > 0.0) ? (w / normalizer) : w;
}

}  // namespace range
}  // namespace resple
