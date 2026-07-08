#pragma once

// ---------------------------------------------------------------------------
// overload_control.h
//
// Dependency-light (std-only) decision cores for running RESPLE on
// resource-limited machines (2026-07-07 overload hardening). Modeled on
// utils/filter_health.h: pure functions/structs, no ROS, unit-tested in
// resple/test/test_overload_control.cpp.
//
//   * latencyShedCount  - how many queued scans to drop so the per-LiDAR
//                         input backlog stays under a data-time budget
//                         ("drop scans, stay real-time" policy)
//   * GapExtrapDamping  - per-knot damping schedule for propRCP's
//                         constant-velocity extrapolation across data gaps
//                         (bounds the post-gap re-entry correction)
// ---------------------------------------------------------------------------

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace resple {
namespace overload {

// How many scans to shed from the FRONT of a per-LiDAR queue so that the
// data-time span kept, measured against `newest_ns` (the stamp of the scan
// being pushed), stays within `max_latency_ns`.
//
// Semantics:
//   * max_latency_ns <= 0  -> feature off, always 0.
//   * Data time, same-LiDAR stamps: independent of bag playback rate, needs
//     no wall clock, and survives sim-time jumps.
//   * Never counts the last element: the newest queued scan is always kept,
//     so the estimator keeps receiving data even when everything is "late".
//   * Non-monotonic input (newest_ns older than the newest queued stamp,
//     e.g. a bag loop restart) -> 0; the existing count-cap and the
//     estimator's own admission gate handle that case.
inline std::size_t latencyShedCount(const std::deque<int64_t>& t_buff,
                                    int64_t newest_ns,
                                    int64_t max_latency_ns) {
  if (max_latency_ns <= 0 || t_buff.empty()) {
    return 0;
  }
  if (newest_ns < t_buff.back()) {
    return 0;  // time went backwards (bag restart); don't shed on bad data
  }
  std::size_t n = 0;
  // Keep at least the newest queued element.
  while (n + 1 < t_buff.size() && newest_ns - t_buff[n] > max_latency_ns) {
    ++n;
  }
  // The (n+1 < size) guard exempts the last element from the loop, but the
  // final element check must still apply when the queue has exactly one
  // stale entry and the incoming scan is fresh: in that case the entry will
  // be superseded by the push that follows, so shedding it is safe as long
  // as the PUSHED scan becomes the newest content. Callers push newest_ns's
  // scan immediately after shedding, so allow shedding the last element too
  // when it exceeds the budget.
  if (n + 1 == t_buff.size() && newest_ns - t_buff[n] > max_latency_ns) {
    ++n;
  }
  return n;
}

// Damping schedule for the RCP constant-velocity extrapolation (Estimator::
// propRCP). k is the knot index within ONE propRCP call: steady-state calls
// add at most ~2 knots, while a shed/dropped-scan gap is fast-forwarded in a
// single call — so `free_knots` (default 3) exempts normal operation and the
// schedule only engages on genuine gaps.
//
//   decay >= 1.0  -> scale(k) == 1.0 for all k (feature off, today's exact
//                    behaviour; callers take the literal legacy expression)
//   decay <  1.0  -> scale(k) = decay^(max(0, k+1-free_knots)); the
//                    extrapolated velocity decays geometrically toward a
//                    hold, so the excursion (and the post-gap correction) is
//                    bounded instead of growing linearly with gap length.
struct GapExtrapDamping {
  double decay = 1.0;
  int free_knots = 3;

  double scale(int k) const {
    if (decay >= 1.0) {
      return 1.0;
    }
    const int excess = k + 1 - free_knots;
    if (excess <= 0) {
      return 1.0;
    }
    return std::pow(decay, excess);
  }
};

}  // namespace overload
}  // namespace resple
