// Unit tests for resple/include/utils/dense_pub.h
//
// ROS-free (std + GoogleTest): the dense odometry back-fill sample-time
// computation (odom/dense_pub_hz, 2026-07-10 dliio-lessons review). Pins the
// contract publishPoseAndTf relies on: strictly-monotonic stamps, bounded
// burst after a gap, exact no-op when disabled or on the first publish.

#include "utils/dense_pub.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

using resple::densepub::firstDenseSampleNs;

namespace {
constexpr int64_t MS = 1'000'000;             // ns per millisecond
constexpr int64_t S = 1'000'000'000;          // ns per second
// A realistic bag epoch (~2023): production stamps live HERE, not near zero.
constexpr int64_t T0 = 1'700'000'000 * S;
constexpr int64_t kNever = std::numeric_limits<int64_t>::min();

// Mirror of the caller's loop: every sample the worker would publish before
// the final edge publish.
std::vector<int64_t> samples(int64_t last, int64_t edge, int64_t interval,
                             int64_t max_backfill) {
  std::vector<int64_t> out;
  for (int64_t t = firstDenseSampleNs(last, edge, interval, max_backfill);
       t < edge; t += interval) {
    out.push_back(t);
  }
  return out;
}
}  // namespace

TEST(DensePub, DisabledReturnsEdge) {
  // interval <= 0 = feature off: the caller publishes only the edge sample,
  // bit-identical to the pre-feature behaviour.
  EXPECT_EQ(firstDenseSampleNs(T0, T0 + 100 * MS, 0, S), T0 + 100 * MS);
  EXPECT_EQ(firstDenseSampleNs(T0, T0 + 100 * MS, -10 * MS, S), T0 + 100 * MS);
  EXPECT_TRUE(samples(T0, T0 + 100 * MS, 0, S).empty());
}

TEST(DensePub, FirstPublishHasNoBackfill) {
  // Sentinel last publish (never published): no history to fill — and no
  // INT64_MIN + interval overflow.
  EXPECT_EQ(firstDenseSampleNs(kNever, T0, 10 * MS, S), T0);
  EXPECT_TRUE(samples(kNever, T0, 10 * MS, S).empty());
}

TEST(DensePub, SteadyStateFillsTheBatchGap) {
  // Typical worker cycle: last publish 100 ms ago, 100 Hz dense rate → nine
  // interior samples, the caller's edge publish makes ten.
  const int64_t last = T0;
  const int64_t edge = T0 + 100 * MS;
  const auto s = samples(last, edge, 10 * MS, S);
  ASSERT_EQ(s.size(), 9u);
  EXPECT_EQ(s.front(), last + 10 * MS);
  EXPECT_EQ(s.back(), edge - 10 * MS);
  // Strictly monotonic and strictly inside (last, edge) — tf2 rejects
  // duplicate stamps and the caller publishes edge itself.
  for (size_t i = 0; i < s.size(); ++i) {
    EXPECT_GT(s[i], last);
    EXPECT_LT(s[i], edge);
    if (i > 0) {
      EXPECT_GT(s[i], s[i - 1]);
    }
  }
}

TEST(DensePub, EdgeLessThanOneIntervalAheadYieldsNothing) {
  // The spline advanced by less than one dense interval: no interior sample.
  EXPECT_EQ(firstDenseSampleNs(T0, T0 + 3 * MS, 10 * MS, S), T0 + 3 * MS);
  EXPECT_TRUE(samples(T0, T0 + 3 * MS, 10 * MS, S).empty());
}

TEST(DensePub, ExactIntervalAdvanceYieldsNothingInterior) {
  // Edge exactly one interval ahead: the only new sample IS the edge.
  EXPECT_EQ(firstDenseSampleNs(T0, T0 + 10 * MS, 10 * MS, S), T0 + 10 * MS);
  EXPECT_TRUE(samples(T0, T0 + 10 * MS, 10 * MS, S).empty());
}

TEST(DensePub, StaleLastPublishIsCappedByMaxBackfill) {
  // Stall / NIS-hold release / bag restart: last publish 30 s stale, cap 1 s
  // → the burst covers only the trailing second, bounded regardless of gap.
  const int64_t edge = T0 + 30 * S;
  const auto s = samples(T0, edge, 10 * MS, S);
  ASSERT_FALSE(s.empty());
  EXPECT_GE(s.front(), edge - S);
  EXPECT_LT(s.back(), edge);
  EXPECT_LE(static_cast<int64_t>(s.size()), S / (10 * MS));
}

TEST(DensePub, SmallEdgeTimesDoNotUnderflow) {
  // Defensive: edge smaller than max_backfill (test rigs, t-near-zero sims).
  // floor clamps at 0 instead of underflowing.
  EXPECT_EQ(firstDenseSampleNs(0, 50 * MS, 10 * MS, S), 10 * MS);
  const auto s = samples(0, 50 * MS, 10 * MS, S);
  ASSERT_EQ(s.size(), 4u);
  EXPECT_EQ(s.front(), 10 * MS);
}
