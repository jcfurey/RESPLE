#pragma once

// ---------------------------------------------------------------------------
// tf_ownership.h
//
// Dependency-light (std-only) decision core for detecting a FOREIGN publisher
// on the TF pair a node owns (2026-07-07 TF-ownership hardening). Modeled on
// utils/overload_control.h / utils/filter_health.h: pure logic, no ROS,
// unit-tested in resple/test/test_tf_ownership.cpp.
//
// Background. tf2 keeps the latest transform per parent->child pair with no
// concept of ownership, and in ROS 2 the wire message carries no publisher
// identity. Two nodes publishing the same pair interleave and the pose
// flickers/jumps; a child frame claimed by two different parents breaks the
// REP-105 tree outright. RESPLE owns odom->base (odom/publish_tf) and the
// Mapping node owns map->odom (map/publish_tf); in a larger stack an external
// EKF (robot_localization), another odometry, or a replayed bag's /tf can
// collide with either.
//
// Detection strategy. The node subscribes to the raw /tf (+ /tf_static)
// topics and feeds every transform whose child matches the watched pair into
// classify(). Because DDS loops a node's own publications back into its own
// subscriptions, self-vs-foreign is decided by exact STAMP matching: the node
// records the nanosecond stamp of every transform it broadcasts
// (notePublished / notePublishedStatic), and an observed same-pair transform
// whose stamp is unknown must have come from someone else. Stamps are data
// timestamps, so the scheme is exact-value: independent of sim time, playback
// rate, and clock jumps.
//
// Threading. classify() runs on the ROS executor (/tf subscription callback);
// notePublished() runs on the publishing thread (worker). The internal mutex
// is a LEAF lock: it is never held while acquiring any other lock, so it
// cannot participate in the estimator's lock-ordering rules (CLAUDE.md).
// Contention is trivial (<= a few hundred short critical sections per second).
// ---------------------------------------------------------------------------

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace resple {
namespace tfown {

enum class Verdict {
  UNRELATED,             // child frame is not the watched one
  SELF,                  // watched pair, stamp we published ourselves
  FOREIGN_SAME_PAIR,     // watched pair, stamp we never published
  FOREIGN_OTHER_PARENT,  // watched child claimed by a DIFFERENT parent
};

class TfOwnershipMonitor {
 public:
  // (Re)arm the monitor for the pair actually broadcast (post-invert_tf).
  // Resets all state — call from on_configure so lifecycle re-cycles start
  // clean.
  void configure(const std::string& parent, const std::string& child) {
    std::lock_guard<std::mutex> lk(m_);
    parent_ = parent;
    child_ = child;
    ring_.fill(kUnsetStamp);
    ring_next_ = 0;
    static_stamps_.clear();
    foreign_same_pair_ = 0;
    foreign_other_parent_ = 0;
    last_foreign_ns_ = kNever;
    pair_seen_ = false;
  }

  // Record the header stamp (ns) of a transform THIS node just broadcast on
  // /tf. Bounded ring: at <= ~100 Hz publishes and <= O(100 ms) DDS + executor
  // delivery latency, 256 slots give seconds of slack before a self stamp can
  // be evicted while its echo is still in flight.
  void notePublished(int64_t stamp_ns) {
    std::lock_guard<std::mutex> lk(m_);
    ring_[ring_next_] = stamp_ns;
    ring_next_ = (ring_next_ + 1) % ring_.size();
  }

  // Record a STATIC broadcast (latched /tf_static). Never evicted: a
  // transient_local re-delivery can arrive arbitrarily late (e.g. on a
  // subscription rebuilt across a lifecycle cycle), long after a ring slot
  // would have been recycled. Bounded — a node latches O(1) static pairs.
  void notePublishedStatic(int64_t stamp_ns) {
    std::lock_guard<std::mutex> lk(m_);
    if (static_stamps_.size() < kMaxStaticStamps) {
      static_stamps_.push_back(stamp_ns);
    }
  }

  // Classify one observed transform. now_ns is a MONOTONIC node-side clock
  // (std::chrono::steady_clock), used only for the freshness window — never
  // compared against the message stamp.
  Verdict classify(const std::string& parent_seen,
                   const std::string& child_seen,
                   int64_t stamp_ns,
                   int64_t now_ns) {
    std::lock_guard<std::mutex> lk(m_);
    if (!frameEq(child_seen, child_)) {
      return Verdict::UNRELATED;
    }
    if (!frameEq(parent_seen, parent_)) {
      ++foreign_other_parent_;
      last_foreign_ns_ = now_ns;
      return Verdict::FOREIGN_OTHER_PARENT;
    }
    pair_seen_ = true;
    for (const int64_t s : ring_) {
      if (s == stamp_ns) return Verdict::SELF;
    }
    for (const int64_t s : static_stamps_) {
      if (s == stamp_ns) return Verdict::SELF;
    }
    ++foreign_same_pair_;
    last_foreign_ns_ = now_ns;
    return Verdict::FOREIGN_SAME_PAIR;
  }

  // True while a foreign observation happened within the trailing window —
  // drives the 'yield' action (suspend own broadcast until the intruder goes
  // quiet) and the diagnostics "conflict active" flag.
  //
  // now_ns comes from the NODE clock (2026-07-08 clock-domain audit): with
  // use_sim_time the quiet window is measured in bag time, so yield/resume
  // behaviour is invariant to playback rate. Jump tolerance: the clock can
  // step backwards (bag loop restart, seeked replay, NTP live), leaving
  // last_foreign in the future. |dt| <= window keeps a small backwards step
  // conservatively "active", while a large jump (loop restart) releases the
  // hold instead of wedging until the clock re-reaches the stale stamp; the
  // next foreign observation re-arms it in the new time base anyway.
  bool foreignActiveWithin(int64_t now_ns, int64_t window_ns) const {
    std::lock_guard<std::mutex> lk(m_);
    if (last_foreign_ns_ == kNever) return false;
    const int64_t dt = now_ns - last_foreign_ns_;
    return dt <= window_ns && dt >= -window_ns;
  }

  // True once the watched pair has been observed at all (self or foreign).
  // With publish_tf=false the ring stays empty, so any same-pair observation
  // sets this — the absence check ("external owner expected but nobody
  // publishes the pair") keys off it.
  bool pairSeen() const {
    std::lock_guard<std::mutex> lk(m_);
    return pair_seen_;
  }

  uint64_t foreignSamePair() const {
    std::lock_guard<std::mutex> lk(m_);
    return foreign_same_pair_;
  }

  uint64_t foreignOtherParent() const {
    std::lock_guard<std::mutex> lk(m_);
    return foreign_other_parent_;
  }

  std::string parent() const {
    std::lock_guard<std::mutex> lk(m_);
    return parent_;
  }

  std::string child() const {
    std::lock_guard<std::mutex> lk(m_);
    return child_;
  }

 private:
  // Stamp value no real transform carries (rcl time is non-negative).
  static constexpr int64_t kUnsetStamp = INT64_MIN;
  static constexpr int64_t kNever = INT64_MIN;
  static constexpr std::size_t kMaxStaticStamps = 8;

  static std::array<int64_t, 256> unsetRing() {
    std::array<int64_t, 256> a;
    a.fill(kUnsetStamp);
    return a;
  }

  // tf2's BufferCore strips a single leading '/' (ROS 1 legacy frame ids);
  // raw publishers may still emit one. Compare under the same normalization
  // so "/odom" and "odom" are one frame — otherwise a legacy-style foreign
  // publisher on our exact pair would evade the same-pair match.
  static bool frameEq(const std::string& a, const std::string& b) {
    const char* pa = a.c_str();
    std::size_t na = a.size();
    const char* pb = b.c_str();
    std::size_t nb = b.size();
    if (na > 0 && pa[0] == '/') { ++pa; --na; }
    if (nb > 0 && pb[0] == '/') { ++pb; --nb; }
    return na == nb && std::memcmp(pa, pb, na) == 0;
  }

  mutable std::mutex m_;  // leaf lock — never held while taking another lock
  std::string parent_;
  std::string child_;
  // Pre-filled with kUnsetStamp so a default-constructed monitor can never
  // false-match a real stamp (configure() also re-fills).
  std::array<int64_t, 256> ring_ = unsetRing();
  std::size_t ring_next_ = 0;
  std::vector<int64_t> static_stamps_;
  uint64_t foreign_same_pair_ = 0;
  uint64_t foreign_other_parent_ = 0;
  int64_t last_foreign_ns_ = kNever;
  bool pair_seen_ = false;
};

}  // namespace tfown
}  // namespace resple
