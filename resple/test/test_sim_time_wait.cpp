// Unit tests for resple/include/utils/sim_time_wait.h
//
// ROS-free (std + GoogleTest): the ROS-time wait tracker used by the
// data-gated timeouts (TF waits, LO IMU wait, init-attitude wait, TF-guard
// absence check). Pins the two sim-time traps: clock-not-started and
// backwards jumps.

#include "utils/sim_time_wait.h"

#include <gtest/gtest.h>

using resple::timeutil::RosTimeWait;

namespace {
constexpr int64_t S = 1'000'000'000;  // ns per second
// A realistic bag epoch (~2023): sim clocks start HERE, not near zero.
constexpr int64_t T0 = 1'700'000'000 * S;
}  // namespace

TEST(RosTimeWait, ZeroClockNeverStartsTheWindow) {
  RosTimeWait w;
  // use_sim_time before the first /clock message: now() reads 0. The window
  // must not start, and certainly must not expire.
  EXPECT_DOUBLE_EQ(w.elapsedSec(0), 0.0);
  EXPECT_DOUBLE_EQ(w.elapsedSec(0), 0.0);
  EXPECT_FALSE(w.started());
}

TEST(RosTimeWait, StartsAtFirstValidSampleNotAtEpochDelta) {
  RosTimeWait w;
  ASSERT_DOUBLE_EQ(w.elapsedSec(0), 0.0);  // pre-/clock tick
  // Clock starts at bag time T0: elapsed must be 0, NOT ~54 years.
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0), 0.0);
  EXPECT_TRUE(w.started());
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 3 * S), 3.0);
}

TEST(RosTimeWait, ThrottledPlaybackScalesWithBagTime) {
  RosTimeWait w;
  // The caller only ever sees bag time; wall rate is irrelevant by
  // construction. 2.5 s of bag time is 2.5 s elapsed, whatever --rate was.
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0), 0.0);
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 5 * S / 2), 2.5);
}

TEST(RosTimeWait, BackwardsJumpRestartsTheWait) {
  RosTimeWait w;
  // Wait starts mid-bag.
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 50 * S), 0.0);
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 58 * S), 8.0);
  // Bag loop restart: the clock lands BEFORE the start stamp — the wait must
  // restart there, not report negative elapsed or wedge on a future stamp.
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 1 * S), 0.0);
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 4 * S), 3.0);
  // A backwards step that stays AFTER the start stamp just shrinks elapsed
  // (never negative): the replayed span is re-waited.
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 2 * S), 1.0);
}

TEST(RosTimeWait, ResetRearms) {
  RosTimeWait w;
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0), 0.0);
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 2 * S), 2.0);
  w.reset();
  EXPECT_FALSE(w.started());
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 10 * S), 0.0);  // fresh window
  EXPECT_DOUBLE_EQ(w.elapsedSec(T0 + 12 * S), 2.0);
}
