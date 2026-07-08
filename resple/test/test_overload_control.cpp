// Unit tests for resple/include/utils/overload_control.h
//
// ROS-free (std + GoogleTest): the latency-shedding decision and the gap-
// extrapolation damping schedule are pure functions, so the overload policy
// is pinned here independent of the node.

#include "utils/overload_control.h"

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <vector>

using resple::overload::GapExtrapDamping;
using resple::overload::latencyShedCount;

namespace {
constexpr int64_t MS = 1'000'000;  // ns per ms
}

// ----------------------------- latencyShedCount -----------------------------

TEST(LatencyShed, OffSwitchAndEmpty) {
  std::deque<int64_t> q = {0, 100 * MS, 200 * MS};
  EXPECT_EQ(latencyShedCount(q, 10'000 * MS, 0), 0u);   // 0 = off
  EXPECT_EQ(latencyShedCount(q, 10'000 * MS, -5), 0u);  // negative = off
  std::deque<int64_t> empty;
  EXPECT_EQ(latencyShedCount(empty, 1000 * MS, 100 * MS), 0u);
}

TEST(LatencyShed, ExactBoundaryDoesNotShed) {
  // Age == budget must NOT shed (strictly-greater comparison).
  std::deque<int64_t> q = {0};
  EXPECT_EQ(latencyShedCount(q, 200 * MS, 200 * MS), 0u);
  EXPECT_EQ(latencyShedCount(q, 200 * MS + 1, 200 * MS), 1u);
}

TEST(LatencyShed, ShedsOnlyTheStalePrefix) {
  // Stamps at 0,50,100,...,400 ms; newest arrival at 450 ms; budget 200 ms.
  std::deque<int64_t> q;
  for (int i = 0; i <= 8; ++i) q.push_back(i * 50 * MS);
  const auto n = latencyShedCount(q, 450 * MS, 200 * MS);
  // Ages: 450,400,...,50 ms. Strictly > 200 ms: 450,400,350,300,250 -> 5.
  EXPECT_EQ(n, 5u);
  // Everything kept is within budget.
  for (std::size_t i = n; i < q.size(); ++i) {
    EXPECT_LE(450 * MS - q[i], 200 * MS);
  }
}

TEST(LatencyShed, WholeQueueStaleIsSheddable) {
  // Every queued scan is older than the budget relative to the incoming one:
  // the incoming push immediately provides fresh content, so all may go.
  std::deque<int64_t> q = {0, 10 * MS, 20 * MS};
  EXPECT_EQ(latencyShedCount(q, 10'000 * MS, 200 * MS), 3u);
}

TEST(LatencyShed, KeepsNewestWhenItIsFresh) {
  // Only the prefix is stale; the newest queued element is within budget and
  // must be kept.
  std::deque<int64_t> q = {0, 950 * MS};
  EXPECT_EQ(latencyShedCount(q, 1000 * MS, 200 * MS), 1u);
}

TEST(LatencyShed, NonMonotonicInputGuard) {
  // Bag loop restart: incoming stamp older than the newest queued stamp.
  std::deque<int64_t> q = {5000 * MS, 5100 * MS};
  EXPECT_EQ(latencyShedCount(q, 100 * MS, 200 * MS), 0u);
}

// ----------------------------- GapExtrapDamping -----------------------------

TEST(GapExtrap, DecayOneIsIdentityForAllK) {
  GapExtrapDamping d;  // decay 1.0
  for (int k = 0; k < 500; ++k) {
    ASSERT_EQ(d.scale(k), 1.0) << k;
  }
  GapExtrapDamping d2{1.5, 3};  // >1 clamps to identity as well
  EXPECT_EQ(d2.scale(100), 1.0);
}

TEST(GapExtrap, FreeKnotExemption) {
  GapExtrapDamping d{0.5, 3};
  // k = 0,1,2 are free (steady-state propRCP adds <= ~2 knots per call).
  EXPECT_EQ(d.scale(0), 1.0);
  EXPECT_EQ(d.scale(1), 1.0);
  EXPECT_EQ(d.scale(2), 1.0);
  EXPECT_DOUBLE_EQ(d.scale(3), 0.5);
  EXPECT_DOUBLE_EQ(d.scale(4), 0.25);
  EXPECT_DOUBLE_EQ(d.scale(5), 0.125);
}

TEST(GapExtrap, RecurrenceVelocityDecaysAndDisplacementBounded) {
  // Simulate the damped propRCP recurrence over a long gap:
  //   p_new = p3 + s_k * (p3 - p2)     (newest knot + damped newest step)
  //
  // NOTE the damped branch deliberately does NOT damp the legacy form
  // 2*p2 - p0 == p2 + (p2 - p0): as s -> 0 that converges to "repeat the
  // position from two knots back", i.e. a 2-cycle oscillation between the
  // last two knot positions — a 50 Hz wobble instead of a hold (verified by
  // simulation while writing this test). p3 + s*(p3 - p2) is identical for
  // constant velocity and converges cleanly to hold-at-newest. With decay<1
  // the per-knot step must decay to ~0 monotonically (never negative) and
  // the total excursion must stay bounded vs. the undamped linear growth.
  for (double decay : {0.5, 0.9}) {
    GapExtrapDamping d{decay, 3};
    // Window of 4 knot positions 1D, constant initial velocity 1 m / knot:
    std::deque<double> w = {0.0, 1.0, 2.0, 3.0};
    double prev_step = 1e9;
    for (int k = 0; k < 400; ++k) {
      const double s = d.scale(k);
      const double p_new = w[3] + s * (w[3] - w[2]);
      const double step = p_new - w[3];
      EXPECT_GE(step, 0.0) << "no backward motion, k=" << k;
      if (k > 3) {
        EXPECT_LE(step, prev_step + 1e-12) << "step must not grow, k=" << k;
      }
      prev_step = step;
      w.pop_front();
      w.push_back(p_new);
    }
    // Velocity ~0 at the end...
    EXPECT_NEAR(w[3] - w[2], 0.0, 1e-6) << "decay=" << decay;
    // ...and the excursion is bounded by free_knots + v/(1-decay), far below
    // the undamped 400 m: 3 + 1/(1-0.5) = 5; 3 + 1/(1-0.9) = 13.
    EXPECT_LT(w[3], 15.0) << "decay=" << decay;
  }
}
