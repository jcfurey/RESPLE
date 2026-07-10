#pragma once

// ---------------------------------------------------------------------------
// dense_pub.h
//
// Dependency-light (std-only) sample-time computation for the dense odometry
// back-fill (odom/dense_pub_hz, 2026-07-10 dliio-lessons review). Modeled on
// utils/sim_time_wait.h: pure function, no ROS, unit-tested in
// resple/test/test_dense_pub.cpp.
//
// Problem: the worker publishes pose/odom/TF once per collectMeasurements
// iteration — normally once per knot (knot_hz, ~100 Hz data time; measured
// live 2026-07-10, R_Campus baseline ≈ 101.7 Hz), so the steady-state chain
// is already dense. But whenever the spline edge advances by MORE than one
// knot between publish calls (propRCP jumping a scan gap, overload shedding
// — HARDENING hazard 69 — or an NIS-hold release), the duplicate gate emits
// ONE pose at the new edge and the published chain has a hole exactly where
// downstream tf2 lookups are most fragile. dliio side-steps this shape by
// publishing from its IMU callback unconditionally; RESPLE's continuous-time
// spline can instead back-fill the hole by interpolation — data-time-stamped,
// never extrapolated. dense_pub_hz also upsamples past knot_hz if a consumer
// wants a finer chain. Latency is unchanged (the samples are historical);
// what is guaranteed is a *floor density* for the published chain.
//
// firstDenseSampleNs computes where that back-fill starts. Guards:
//   * never published yet (last_pub_ns == INT64_MIN, the same sentinel
//     RESPLE.cpp keeps in last_pose_pub_time_ns_): nothing to fill — the
//     first batch publishes the edge only, no burst of boundary-clamped
//     startup poses;
//   * stale last publish (stall, NIS-hold release, bag restart): the fill is
//     capped at max_backfill_ns of history so the burst stays bounded;
//   * disabled (interval_ns <= 0): return edge_ns (caller publishes the edge
//     sample only — bit-identical to the pre-feature behaviour);
//   * all samples land strictly AFTER last_pub_ns and strictly BEFORE
//     edge_ns; the caller always publishes edge_ns itself last, so stamps
//     stay strictly monotonic (tf2 rejects duplicate stamps).
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cstdint>
#include <limits>

namespace resple {
namespace densepub {

// First back-fill sample time in (last_pub_ns, edge_ns), stepping by
// interval_ns, with at most max_backfill_ns of history covered. Returns
// edge_ns when there is nothing to back-fill (disabled, never published yet,
// or the edge is less than one interval ahead of the last publish).
inline int64_t firstDenseSampleNs(int64_t last_pub_ns, int64_t edge_ns,
                                  int64_t interval_ns,
                                  int64_t max_backfill_ns) {
  if (interval_ns <= 0 || max_backfill_ns < 0 ||
      last_pub_ns == std::numeric_limits<int64_t>::min()) {
    return edge_ns;
  }
  // Oldest admissible sample. Written as a subtraction guard, not
  // edge - max_backfill directly: edge_ns is a real bag/system stamp
  // (~1.7e18) in production but tests use small values, and INT64 underflow
  // is UB.
  const int64_t floor_ns = edge_ns > max_backfill_ns ? edge_ns - max_backfill_ns : 0;
  // last_pub_ns can be arbitrarily stale after a stall; clamp BEFORE adding
  // interval_ns so the addition cannot overflow.
  const int64_t last_eff = std::max(last_pub_ns, floor_ns - interval_ns);
  const int64_t start = last_eff + interval_ns;
  return start < edge_ns ? start : edge_ns;
}

}  // namespace densepub
}  // namespace resple
