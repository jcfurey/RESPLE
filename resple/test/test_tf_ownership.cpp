// Unit tests for resple/include/utils/tf_ownership.h
//
// ROS-free (std + GoogleTest): the self-vs-foreign TF classification is pure
// logic, so the ownership-guard policy is pinned here independent of the node.

#include "utils/tf_ownership.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

using resple::tfown::TfOwnershipMonitor;
using resple::tfown::Verdict;

namespace {
constexpr int64_t S = 1'000'000'000;  // ns per second
}

TEST(TfOwnership, UnrelatedChildIgnored) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  // Other frames flowing on /tf (URDF chain, sensor extrinsics) never count.
  EXPECT_EQ(m.classify("base_footprint", "base_link", 1 * S, 0), Verdict::UNRELATED);
  EXPECT_EQ(m.classify("map", "odom", 1 * S, 0), Verdict::UNRELATED);
  EXPECT_EQ(m.foreignSamePair(), 0u);
  EXPECT_EQ(m.foreignOtherParent(), 0u);
  EXPECT_FALSE(m.pairSeen());
}

TEST(TfOwnership, SelfEchoByStampMatch) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  m.notePublished(42 * S);
  // DDS loops our own /tf publication back: exact stamp -> SELF, no count.
  EXPECT_EQ(m.classify("odom", "base_footprint", 42 * S, 0), Verdict::SELF);
  EXPECT_EQ(m.foreignSamePair(), 0u);
  EXPECT_TRUE(m.pairSeen());
}

TEST(TfOwnership, ForeignSamePairByStampMiss) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  m.notePublished(42 * S);
  // Same pair, a stamp we never produced: another publisher (EKF, DLIO,
  // replayed bag) is fighting us for the pair.
  EXPECT_EQ(m.classify("odom", "base_footprint", 43 * S, 7), Verdict::FOREIGN_SAME_PAIR);
  EXPECT_EQ(m.foreignSamePair(), 1u);
  EXPECT_TRUE(m.foreignActiveWithin(7, 0));
}

TEST(TfOwnership, ForeignOtherParentBreaksTree) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_link");
  // Someone publishes base_footprint->base_link (URDF static) while we
  // publish odom->base_link: the child has two parents, the tree is broken
  // even though nobody collides on our exact pair.
  EXPECT_EQ(m.classify("base_footprint", "base_link", 1 * S, 3), Verdict::FOREIGN_OTHER_PARENT);
  EXPECT_EQ(m.foreignOtherParent(), 1u);
  EXPECT_EQ(m.foreignSamePair(), 0u);
  EXPECT_TRUE(m.foreignActiveWithin(3, 0));
  // Exact-pair tracking is separate: not seen yet.
  EXPECT_FALSE(m.pairSeen());
}

TEST(TfOwnership, EmptyRingNeverMatchesStampZero) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  // A (malformed) transform with stamp 0 must not match unset ring slots.
  EXPECT_EQ(m.classify("odom", "base_footprint", 0, 0), Verdict::FOREIGN_SAME_PAIR);
}

TEST(TfOwnership, RingEvictsOldestAfterWraparound) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  // Publish 300 stamps into the 256-slot ring: stamp 0..43 are evicted.
  for (int i = 0; i < 300; ++i) m.notePublished(i);
  EXPECT_EQ(m.classify("odom", "base_footprint", 299, 0), Verdict::SELF);
  EXPECT_EQ(m.classify("odom", "base_footprint", 44, 0), Verdict::SELF);
  // Documented bound: an echo older than the ring depth reads as foreign.
  EXPECT_EQ(m.classify("odom", "base_footprint", 0, 0), Verdict::FOREIGN_SAME_PAIR);
}

TEST(TfOwnership, StaticStampsAreNeverEvicted) {
  TfOwnershipMonitor m;
  m.configure("map", "odom");
  m.notePublishedStatic(5 * S);  // Mapping's identity latch on /tf_static
  for (int i = 0; i < 1000; ++i) m.notePublished(100 * S + i);
  // A transient_local re-delivery of our own latch, arbitrarily late, is SELF.
  EXPECT_EQ(m.classify("map", "odom", 5 * S, 0), Verdict::SELF);
}

TEST(TfOwnership, FreshnessWindowExpires) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  m.classify("odom", "base_footprint", 1 * S, /*now=*/10 * S);
  EXPECT_TRUE(m.foreignActiveWithin(12 * S, 5 * S));   // 2 s ago, 5 s window
  EXPECT_TRUE(m.foreignActiveWithin(15 * S, 5 * S));   // boundary inclusive
  EXPECT_FALSE(m.foreignActiveWithin(16 * S, 5 * S));  // quiet long enough
  // Never-foreign monitor is never active.
  TfOwnershipMonitor quiet;
  quiet.configure("odom", "base_footprint");
  EXPECT_FALSE(quiet.foreignActiveWithin(0, INT64_MAX / 2));
}

TEST(TfOwnership, PairSeenDrivesAbsenceCheck) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  // publish_tf=false case: we never notePublished, an external EKF owns the
  // pair. Its transforms classify as FOREIGN_SAME_PAIR (the node ignores the
  // verdict in that mode) but pairSeen() flips — the absence check stays
  // quiet.
  EXPECT_FALSE(m.pairSeen());
  m.classify("odom", "base_footprint", 9 * S, 0);
  EXPECT_TRUE(m.pairSeen());
}

TEST(TfOwnership, ConfigureResetsEverything) {
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  m.notePublished(1 * S);
  m.notePublishedStatic(2 * S);
  m.classify("odom", "base_footprint", 3 * S, 5);   // foreign
  m.classify("map", "base_footprint", 3 * S, 5);    // other parent
  ASSERT_GT(m.foreignSamePair() + m.foreignOtherParent(), 0u);
  // Lifecycle re-cycle re-arms clean (possibly with new frame ids).
  m.configure("odom", "base_link");
  EXPECT_EQ(m.foreignSamePair(), 0u);
  EXPECT_EQ(m.foreignOtherParent(), 0u);
  EXPECT_FALSE(m.pairSeen());
  EXPECT_FALSE(m.foreignActiveWithin(6, INT64_MAX / 2));
  // Old self stamps must not leak into the new configuration.
  EXPECT_EQ(m.classify("odom", "base_link", 1 * S, 0), Verdict::FOREIGN_SAME_PAIR);
  EXPECT_EQ(m.parent(), "odom");
  EXPECT_EQ(m.child(), "base_link");
}

TEST(TfOwnership, LeadingSlashNormalization) {
  // tf2 strips one leading '/' (ROS 1 legacy ids); a foreign publisher
  // emitting "/odom"->"/base_footprint" is the SAME pair and must be caught,
  // not evade the match as UNRELATED.
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  m.notePublished(1 * S);
  EXPECT_EQ(m.classify("/odom", "/base_footprint", 1 * S, 0), Verdict::SELF);
  EXPECT_EQ(m.classify("/odom", "/base_footprint", 2 * S, 0), Verdict::FOREIGN_SAME_PAIR);
  EXPECT_EQ(m.classify("/map", "base_footprint", 1 * S, 0), Verdict::FOREIGN_OTHER_PARENT);
  // Normalization is one slash only — a genuinely different frame stays out.
  EXPECT_EQ(m.classify("odom", "/base", 1 * S, 0), Verdict::UNRELATED);
}

TEST(TfOwnership, InvertedPairIsJustTheWatchedPair) {
  // invert_tf swaps the broadcast direction; the node configures the monitor
  // with the POST-inversion pair, so base->odom is then the exact pair and
  // odom->base is "other parent" territory only if child matches.
  TfOwnershipMonitor m;
  m.configure("base_footprint", "odom");
  m.notePublished(1 * S);
  EXPECT_EQ(m.classify("base_footprint", "odom", 1 * S, 0), Verdict::SELF);
  EXPECT_EQ(m.classify("map", "odom", 1 * S, 0), Verdict::FOREIGN_OTHER_PARENT);
  EXPECT_EQ(m.classify("odom", "base_footprint", 1 * S, 0), Verdict::UNRELATED);
}
