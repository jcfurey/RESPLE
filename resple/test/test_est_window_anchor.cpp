// End-to-end regression for the est_window replica ANCHOR (2026-07-26 hunt).
//
// The est_window protocol ships per window: absolute knot POSITIONS, per-knot
// orientation DELTAS, three "idle" knots and a start_q anchor. A fresh receiver
// (the Mapping node after a start_time handshake) rebuilds its idle slots from
// those and then chains every knot quaternion from q_idle[2]:
//
//     q_knots[0] = q_idle[2] * exp(ort_delta[0])          (addOneStateKnot)
//
// so q_idle[2] must be the ABSOLUTE orientation of the knot immediately before
// the window's first knot (absolute index start_idx - 1).
//
// getSplineMsg used to ship the sender's OWN idle slots, which describe absolute
// knots (num_knots_pruned_-3 .. num_knots_pruned_-1). RESPLE is unpruned at
// startup, so that anchor described the spline ORIGIN — while the first window
// the Mapping node ACCEPTS begins at absolute index T > 0, because every window
// published before the start_time handshake completes is rejected (getEstCallback
// early-returns on !if_init_succeed) and getSplineMsg's `last_start_idx_ + 1`
// throttle walks start_idx up by one per publish in the meantime.
//
// Result: the replica was short the rotation accumulated over the skipped knots
// 0..T-1. Position self-heals (absolute control points, so only the first two
// knot intervals blend the origin); orientation does NOT, because every later
// knot chains from the mis-anchored knot 0 — the whole replica map/path/odom
// stays rotated for the rest of the session.
//
// The tests below drive the REAL getSplineMsg and reconstruct the receiver
// exactly as Mapping::getEstCallback does.

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "SplineState.h"

namespace {

constexpr int64_t kDtNs = 10000000LL;  // knot_hz = 100

// Quater::exp(v) = [cos|v|, sinc(|v|) v] -> the rotation angle is 2*|v|.
double deltaAngleDeg(const Eigen::Vector3d& d)
{
    return 2.0 * d.norm() * 180.0 / M_PI;
}

// Rebuild a window SplineState from a Spline message, mirroring
// Mapping::getEstCallback (addOneStateKnot for every knot, THEN setIdles).
SplineState windowFromMsg(const estimate_msgs::msg::Spline& msg)
{
    SplineState w;
    w.init(msg.dt, 0, msg.start_t, static_cast<int>(msg.start_idx));
    for (const auto& knot : msg.knots) {
        w.addOneStateKnot(
            Eigen::Vector3d(knot.position.x, knot.position.y, knot.position.z),
            Eigen::Vector3d(knot.orientation_del.x, knot.orientation_del.y,
                            knot.orientation_del.z));
    }
    const Eigen::Quaterniond q_idle0(msg.start_q.w, msg.start_q.x, msg.start_q.y,
                                     msg.start_q.z);
    for (int i = 0; i < 3 && i < static_cast<int>(msg.idles.size()); ++i) {
        const auto& idle = msg.idles[i];
        w.setIdles(i,
                   Eigen::Vector3d(idle.position.x, idle.position.y, idle.position.z),
                   Eigen::Vector3d(idle.orientation_del.x, idle.orientation_del.y,
                                   idle.orientation_del.z),
                   q_idle0);
    }
    return w;
}

// Grow a sender over `n_knots`, publishing a window after every knot (as the
// RESPLE worker does). Returns the LAST published message — the first one the
// Mapping node would accept once its handshake completes.
estimate_msgs::msg::Spline growAndPublish(SplineState& sender, int64_t n_knots,
                                          const Eigen::Vector3d& per_knot_step,
                                          const Eigen::Vector3d& per_knot_delta)
{
    estimate_msgs::msg::Spline msg;
    for (int64_t k = 0; k < n_knots; ++k) {
        sender.addOneStateKnot(per_knot_step * double(k), per_knot_delta);
        msg = estimate_msgs::msg::Spline{};
        sender.getSplineMsg(msg, sender.totalKnots());
    }
    return msg;
}

struct Deviation {
    double worst_deg = 0.0;
    double worst_m = 0.0;
    double final_deg = 0.0;
};

Deviation compare(const SplineState& sender, const SplineState& recv)
{
    Deviation d;
    for (int64_t t = recv.minTimeNs(); t <= recv.maxTimeNs(); t += kDtNs / 4) {
        Eigen::Quaterniond qs, qr;
        sender.itpQuaternion(t, &qs);
        recv.itpQuaternion(t, &qr);
        d.final_deg = qs.angularDistance(qr) * 180.0 / M_PI;
        d.worst_deg = std::max(d.worst_deg, d.final_deg);
        d.worst_m = std::max(d.worst_m,
                             (sender.itpPosition(t) - recv.itpPosition(t)).norm());
    }
    return d;
}

}  // namespace

// The handshake skips T knots while the platform is MOVING and ROTATING (the
// documented init_attitude_source: tf / tf_gravity_check mode, or a RESPLE
// respawn mid-motion). The replica must reproduce the sender's absolute pose,
// not merely its deltas.
TEST(EstWindowAnchor, StartupSkipPreservesSenderPose)
{
    const int64_t T = 50;                         // 0.5 s handshake at 100 Hz
    const Eigen::Vector3d step(0.01, 0.0, 0.0);   // 1 m/s
    const Eigen::Vector3d dyaw(0.0, 0.0, 0.005);  // 0.573 deg/knot

    SplineState sender;
    sender.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity());
    const auto msg = growAndPublish(sender, T + 5, step, dyaw);

    // The handshake must actually have walked start_idx off the origin,
    // otherwise this test proves nothing.
    ASSERT_GT(msg.start_idx, 3u) << "window did not skip past the origin";
    const double skipped_deg = deltaAngleDeg(dyaw) * double(msg.start_idx);
    ASSERT_GT(skipped_deg, 10.0) << "test must skip a meaningful rotation";

    SplineState window = windowFromMsg(msg);
    SplineState recv;
    recv.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
              Eigen::Quaterniond::Identity());
    recv.setTimeIntervalNs(window.getKnotTimeIntervalNs());
    recv.updateKnots(&window);

    // Time axis (fixed by the startup-prune change): knot 0 sits at the
    // sender's knot start_idx.
    EXPECT_EQ(recv.getKnotTimeNs(0),
              sender.getKnotTimeNs(static_cast<size_t>(msg.start_idx)));
    EXPECT_EQ(recv.numKnotsPruned(), static_cast<int64_t>(msg.start_idx));

    const Deviation d = compare(sender, recv);
    EXPECT_LT(d.worst_deg, 1e-6)
        << "replica orientation mis-anchored by " << d.worst_deg
        << " deg (rotation over the " << msg.start_idx << " skipped knots was "
        << skipped_deg << " deg); error at the window's far end was "
        << d.final_deg << " deg — a mis-anchored chain never recovers";
    EXPECT_LT(d.worst_m, 1e-9)
        << "replica position off by " << d.worst_m
        << " m — the first two knot intervals blended the spline origin";
}

// Same handshake skip, but with the platform purely translating: isolates the
// position half (the first two knot intervals must not blend the origin).
TEST(EstWindowAnchor, StartupSkipPreservesPositionNoRotation)
{
    const int64_t T = 50;
    const Eigen::Vector3d step(0.01, 0.0, 0.0);

    SplineState sender;
    sender.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity());
    const auto msg = growAndPublish(sender, T + 5, step, Eigen::Vector3d::Zero());
    ASSERT_GT(msg.start_idx, 3u);

    SplineState window = windowFromMsg(msg);
    SplineState recv;
    recv.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
              Eigen::Quaterniond::Identity());
    recv.setTimeIntervalNs(window.getKnotTimeIntervalNs());
    recv.updateKnots(&window);

    const Deviation d = compare(sender, recv);
    EXPECT_LT(d.worst_m, 1e-9) << "replica position off by " << d.worst_m << " m";
    EXPECT_LT(d.worst_deg, 1e-6);
}

// SHORT handshake blackout: Mapping's uninitialized spin is a single 100 ms
// worker iteration while RESPLE publishes ~30-40 windows/s, so the first
// accepted window very often has start_idx of just 1 or 2. Those land where
// fewer than three real knots precede start_idx, so the anchor is assembled
// PER SLOT from a mix of real knots and idle slots — getting that mapping
// wrong costs a permanent 1-2 knots of rotation (0.57 deg / 1.15 deg here).
TEST(EstWindowAnchor, ShortBlackoutAnchorsPerSlot)
{
    const Eigen::Vector3d step(0.01, 0.0, 0.0);
    const Eigen::Vector3d dyaw(0.0, 0.0, 0.005);

    // getSplineMsg walks start_idx up by one per publish, so publishing after
    // exactly k knots yields a window with start_idx == k-1 for small k.
    for (int64_t n_knots = 2; n_knots <= 8; ++n_knots) {
        SplineState sender;
        sender.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
                    Eigen::Quaterniond::Identity());
        const auto msg = growAndPublish(sender, n_knots, step, dyaw);

        SplineState window = windowFromMsg(msg);
        SplineState recv;
        recv.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
                  Eigen::Quaterniond::Identity());
        recv.setTimeIntervalNs(window.getKnotTimeIntervalNs());
        recv.updateKnots(&window);

        const Deviation d = compare(sender, recv);
        EXPECT_LT(d.worst_deg, 1e-6)
            << "n_knots=" << n_knots << " start_idx=" << msg.start_idx
            << ": replica orientation off by " << d.worst_deg << " deg";
        EXPECT_LT(d.worst_m, 1e-9)
            << "n_knots=" << n_knots << " start_idx=" << msg.start_idx
            << ": replica position off by " << d.worst_m << " m";
    }
}

// A stationary platform (the default gravity-init path) must be unaffected —
// this is what keeps the shipped default configuration correct.
TEST(EstWindowAnchor, StationaryStartupIsExact)
{
    SplineState sender;
    sender.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity());
    const auto msg = growAndPublish(sender, 55, Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d::Zero());

    SplineState window = windowFromMsg(msg);
    SplineState recv;
    recv.init(kDtNs, 0, 0, 0, Eigen::Vector3d::Zero(),
              Eigen::Quaterniond::Identity());
    recv.setTimeIntervalNs(window.getKnotTimeIntervalNs());
    recv.updateKnots(&window);

    const Deviation d = compare(sender, recv);
    EXPECT_LT(d.worst_deg, 1e-9);
    EXPECT_LT(d.worst_m, 1e-9);
}
