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
  // Overfill the ring by 50: the oldest 50 stamps are evicted.
  const int64_t n = static_cast<int64_t>(TfOwnershipMonitor::kRingSlots);
  for (int64_t i = 0; i < n + 50; ++i) m.notePublished(i);
  EXPECT_EQ(m.classify("odom", "base_footprint", n + 49, 0), Verdict::SELF);
  EXPECT_EQ(m.classify("odom", "base_footprint", 50, 0), Verdict::SELF);
  // Documented bound: an echo older than the ring depth reads as foreign.
  EXPECT_EQ(m.classify("odom", "base_footprint", 0, 0), Verdict::FOREIGN_SAME_PAIR);
  EXPECT_EQ(m.classify("odom", "base_footprint", 49, 0), Verdict::FOREIGN_SAME_PAIR);
}

TEST(TfOwnership, WorstCaseDensePublishBurstStaysSelf) {
  // Sizing invariant against the dense back-fill (odom/dense_pub_hz): the
  // worst burst is the 1 kHz clamp x the 1 s history cap = 1001 stamps
  // published in one tight worker loop, whose DDS loopback echoes may all be
  // processed only AFTER the burst finishes. Every stamp must still be in
  // the ring at that point — an evicted own-stamp would classify
  // FOREIGN_SAME_PAIR: ERROR spam in 'warn' mode, and in 'yield' mode a
  // self-inflicted TF hole right after the stall the burst was back-filling.
  constexpr int64_t kWorstBurst = 1001;
  static_assert(kWorstBurst <= static_cast<int64_t>(TfOwnershipMonitor::kRingSlots),
                "dense back-fill worst burst must fit the self-stamp ring");
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  const int64_t MS_ = 1'000'000;
  for (int64_t i = 0; i < kWorstBurst; ++i) {
    m.notePublished(1 * S + i * MS_);  // 1 kHz dense samples
  }
  // Echoes drain afterwards: every one must be SELF, zero foreign counts.
  for (int64_t i = 0; i < kWorstBurst; ++i) {
    ASSERT_EQ(m.classify("odom", "base_footprint", 1 * S + i * MS_, 0),
              Verdict::SELF)
        << "burst stamp " << i << " evicted before its echo classified";
  }
  EXPECT_EQ(m.foreignSamePair(), 0u);
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

TEST(TfOwnership, FreshnessSurvivesClockJumps) {
  // now comes from the node clock (sim time on bags), which can step
  // backwards on a bag loop restart. A small backwards step stays
  // conservatively active; a large jump releases instead of wedging.
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  m.classify("odom", "base_footprint", 1 * S, /*now=*/100 * S);
  EXPECT_TRUE(m.foreignActiveWithin(98 * S, 5 * S));   // -2 s: still active
  EXPECT_FALSE(m.foreignActiveWithin(10 * S, 5 * S));  // loop restart: release
  // Re-observation in the new time base re-arms normally.
  m.classify("odom", "base_footprint", 2 * S, /*now=*/11 * S);
  EXPECT_TRUE(m.foreignActiveWithin(12 * S, 5 * S));
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

TEST(TfOwnership, StaticForeignClaimIsSticky) {
  // A foreign transform arriving via /tf_static persists in every running
  // consumer's tf2 buffer forever — the conflict cannot "go quiet", so the
  // sticky flag must hold regardless of the freshness window. This also
  // covers the sim-time corner: delivery before the first /clock message
  // (now == 0) would age out of the window instantly, but stickiness won't.
  TfOwnershipMonitor m;
  m.configure("map", "odom");
  EXPECT_FALSE(m.foreignStaticSeen());
  // Pre-/clock delivery (now == 0), static foreign same-pair.
  EXPECT_EQ(m.classify("map", "odom", 7 * S, /*now=*/0, /*from_static=*/true),
            Verdict::FOREIGN_SAME_PAIR);
  EXPECT_TRUE(m.foreignStaticSeen());
  // Freshness window long expired — stickiness unaffected.
  EXPECT_FALSE(m.foreignActiveWithin(1000 * S, 5 * S));
  EXPECT_TRUE(m.foreignStaticSeen());
  // Static other-parent claims stick too; dynamic foreigns never set it.
  TfOwnershipMonitor m2;
  m2.configure("map", "odom");
  m2.classify("earth", "odom", 1 * S, 0, /*from_static=*/true);
  EXPECT_TRUE(m2.foreignStaticSeen());
  TfOwnershipMonitor m3;
  m3.configure("map", "odom");
  m3.classify("map", "odom", 1 * S, 0, /*from_static=*/false);
  EXPECT_FALSE(m3.foreignStaticSeen());
  // Our OWN static latch (whitelisted stamp) must not stick.
  TfOwnershipMonitor m4;
  m4.configure("map", "odom");
  m4.notePublishedStatic(3 * S);
  EXPECT_EQ(m4.classify("map", "odom", 3 * S, 0, /*from_static=*/true),
            Verdict::SELF);
  EXPECT_FALSE(m4.foreignStaticSeen());
  // configure() re-arms (lifecycle re-cycle).
  m.configure("map", "odom");
  EXPECT_FALSE(m.foreignStaticSeen());
}

TEST(TfOwnership, LastPairSeenDrivesAbsenceRearm) {
  // publish_tf=false: the external owner published for a while, then died.
  // lastPairSeenNs going stale is the "owner stopped" signal, distinct from
  // pairSeen()==false ("never wired up").
  TfOwnershipMonitor m;
  m.configure("odom", "base_footprint");
  EXPECT_EQ(m.lastPairSeenNs(), 0);  // never
  m.classify("odom", "base_footprint", 1 * S, /*now=*/10 * S);
  EXPECT_EQ(m.lastPairSeenNs(), 10 * S);
  m.classify("odom", "base_footprint", 2 * S, /*now=*/12 * S);
  EXPECT_EQ(m.lastPairSeenNs(), 12 * S);
  // Other-parent observations are NOT the watched pair — no update.
  m.classify("map", "base_footprint", 3 * S, /*now=*/20 * S);
  EXPECT_EQ(m.lastPairSeenNs(), 12 * S);
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
