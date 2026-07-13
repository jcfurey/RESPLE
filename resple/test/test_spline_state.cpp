// Unit tests for SplineState sliding-window knot pruning (HARDENING Phase 3.1)
// and the est_window message protocol it must preserve.
//
// ROS-free: built standalone against the stub estimate_msgs headers in
// test/stubs/ (see resple/test/CMakeLists.txt), and against the real generated
// messages in the colcon BUILD_TESTING path. Only message FIELDS are accessed,
// so the same source compiles in both contexts.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "SplineState.h"

namespace
{

constexpr int64_t kDtNs = 10000000;  // 10 ms == 100 Hz knot rate
constexpr int64_t kStartNs = 5000000000;

// Deterministic, smooth-ish synthetic trajectory: position wanders, rotation
// accumulates small per-knot deltas (valid for the cumulative-form spline).
Eigen::Vector3d knotPos(int i)
{
    return Eigen::Vector3d(0.05 * i + 0.3 * std::sin(0.07 * i),
                           0.2 * std::cos(0.05 * i),
                           0.01 * i);
}

Eigen::Vector3d knotOrtDel(int i)
{
    return Eigen::Vector3d(0.02 * std::sin(0.11 * i),
                           0.015 * std::cos(0.13 * i),
                           0.01 * std::sin(0.05 * i + 0.4));
}

SplineState makeSpline(int num_knots)
{
    SplineState s;
    s.init(kDtNs, 0, kStartNs);
    for (int i = 0; i < num_knots; i++) {
        s.addOneStateKnot(knotPos(i), knotOrtDel(i));
    }
    return s;
}

struct PoseSample
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    Eigen::Vector3d vel;
    Eigen::Vector3d w;
};

PoseSample sampleAt(const SplineState& s, int64_t t_ns)
{
    PoseSample out;
    out.pos = s.itpPosition(t_ns);
    out.vel = s.itpPosition<1>(t_ns);
    s.itpQuaternion(t_ns, &out.q, &out.w);
    return out;
}

}  // namespace

// --- Core property: pruning must not change interpolation on the retained
// --- range. The idle-window slide makes the evaluation inputs identical, so
// --- equality here is exact (not approximate).

TEST(SplinePrune, InterpolationIdenticalOnRetainedRange)
{
    SplineState full = makeSpline(200);
    SplineState pruned = makeSpline(200);
    const int64_t n_pruned = pruned.pruneFrontKnots(60);
    ASSERT_EQ(n_pruned, 140);
    ASSERT_EQ(pruned.numKnots(), 60);
    ASSERT_EQ(pruned.totalKnots(), 200);
    ASSERT_EQ(pruned.minTimeNs(), kStartNs + 140 * kDtNs);
    ASSERT_EQ(pruned.maxTimeNs(), full.maxTimeNs());

    // Sweep the retained range densely, including both boundaries and
    // off-grid times.
    for (int64_t t = pruned.minTimeNs(); t <= pruned.maxTimeNs(); t += kDtNs / 3) {
        PoseSample a = sampleAt(full, t);
        PoseSample b = sampleAt(pruned, t);
        EXPECT_EQ(a.pos, b.pos) << "position differs at t=" << t;
        EXPECT_EQ(a.vel, b.vel) << "velocity differs at t=" << t;
        EXPECT_EQ(a.q.coeffs(), b.q.coeffs()) << "quaternion differs at t=" << t;
        EXPECT_EQ(a.w, b.w) << "angular velocity differs at t=" << t;
    }
    PoseSample a = sampleAt(full, pruned.maxTimeNs());
    PoseSample b = sampleAt(pruned, pruned.maxTimeNs());
    EXPECT_EQ(a.pos, b.pos);
    EXPECT_EQ(a.q.coeffs(), b.q.coeffs());
}

TEST(SplinePrune, ItpPoseMatchesAfterPrune)
{
    SplineState full = makeSpline(120);
    SplineState pruned = makeSpline(120);
    pruned.pruneFrontKnots(30);

    for (int64_t t = pruned.minTimeNs(); t <= pruned.maxTimeNs(); t += kDtNs / 2) {
        Eigen::Vector3d p_full, p_pruned;
        Eigen::Quaterniond q_full, q_pruned;
        Jacobian J_p_full, J_p_pruned;
        Jacobian43 J_q_full, J_q_pruned;
        full.itpPose(t, &p_full, &J_p_full, &q_full, &J_q_full);
        pruned.itpPose(t, &p_pruned, &J_p_pruned, &q_pruned, &J_q_pruned);
        EXPECT_EQ(p_full, p_pruned) << "itpPose position differs at t=" << t;
        EXPECT_EQ(q_full.coeffs(), q_pruned.coeffs()) << "itpPose quaternion differs at t=" << t;
        // Jacobian VALUES must match wherever the pruned spline's window is
        // made of regular knots; start_idx is deque-local so it shifts by the
        // pruned count. In the first 3 knot intervals after the boundary some
        // control points are idles (not estimated state), so the pruned
        // Jacobian is legitimately shorter there — the IEKF only evaluates
        // Jacobians at the spline TAIL, never near the pruned boundary.
        if (t < pruned.minTimeNs() + 3 * kDtNs) {
            continue;
        }
        ASSERT_EQ(J_p_full.d_val_d_knot.size(), J_p_pruned.d_val_d_knot.size());
        for (size_t k = 0; k < J_p_full.d_val_d_knot.size(); k++) {
            EXPECT_EQ(J_p_full.d_val_d_knot[k], J_p_pruned.d_val_d_knot[k]);
        }
        ASSERT_EQ(J_q_full.d_val_d_knot.size(), J_q_pruned.d_val_d_knot.size());
        for (size_t k = 0; k < J_q_full.d_val_d_knot.size(); k++) {
            EXPECT_EQ(J_q_full.d_val_d_knot[k], J_q_pruned.d_val_d_knot[k]);
        }
        EXPECT_EQ(static_cast<int64_t>(J_p_full.start_idx),
                  static_cast<int64_t>(J_p_pruned.start_idx) + pruned.numKnotsPruned());
    }
}

// Pruning fewer than 3 knots exercises the idle-slot reuse path (the new idle
// window then still contains 1-2 of the OLD idle entries).
TEST(SplinePrune, SmallPruneReusesOldIdles)
{
    for (int prune_n = 1; prune_n <= 3; prune_n++) {
        SplineState full = makeSpline(20);
        SplineState pruned = makeSpline(20);
        ASSERT_EQ(pruned.pruneFrontKnots(20 - prune_n), prune_n) << "prune_n=" << prune_n;
        for (int64_t t = pruned.minTimeNs(); t <= pruned.maxTimeNs(); t += kDtNs / 4) {
            PoseSample a = sampleAt(full, t);
            PoseSample b = sampleAt(pruned, t);
            EXPECT_EQ(a.pos, b.pos) << "prune_n=" << prune_n << " t=" << t;
            EXPECT_EQ(a.q.coeffs(), b.q.coeffs()) << "prune_n=" << prune_n << " t=" << t;
        }
    }
}

TEST(SplinePrune, RepeatedPruneEqualsSinglePrune)
{
    SplineState once = makeSpline(300);
    SplineState steps = makeSpline(300);
    once.pruneFrontKnots(50);
    steps.pruneFrontKnots(200);
    steps.pruneFrontKnots(120);
    steps.pruneFrontKnots(50);
    ASSERT_EQ(once.numKnots(), steps.numKnots());
    ASSERT_EQ(once.totalKnots(), steps.totalKnots());
    ASSERT_EQ(once.minTimeNs(), steps.minTimeNs());
    for (int64_t t = once.minTimeNs(); t <= once.maxTimeNs(); t += kDtNs / 2) {
        PoseSample a = sampleAt(once, t);
        PoseSample b = sampleAt(steps, t);
        EXPECT_EQ(a.pos, b.pos);
        EXPECT_EQ(a.q.coeffs(), b.q.coeffs());
    }
}

TEST(SplinePrune, NoOpAndFloorBehaviour)
{
    SplineState s = makeSpline(10);
    EXPECT_EQ(s.pruneFrontKnots(10), 0);   // nothing to prune
    EXPECT_EQ(s.pruneFrontKnots(500), 0);  // keep > size
    EXPECT_EQ(s.pruneFrontKnots(0), 2);    // floor clamps keep to 8
    EXPECT_EQ(s.numKnots(), 8);
    EXPECT_EQ(s.pruneFrontKnots(0), 0);    // already at floor
    EXPECT_EQ(s.totalKnots(), 10);
}

TEST(SplinePrune, InitResetsPruneStateAndKnots)
{
    SplineState s = makeSpline(50);
    s.pruneFrontKnots(20);
    ASSERT_EQ(s.numKnotsPruned(), 30);
    s.init(kDtNs, 0, kStartNs);
    EXPECT_EQ(s.numKnots(), 0);
    EXPECT_EQ(s.totalKnots(), 0);
    EXPECT_EQ(s.numKnotsPruned(), 0);
    // Re-init must also drop the old deque contents: the first knot added
    // after re-init is knot 0 again.
    s.addOneStateKnot(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d::Zero());
    EXPECT_EQ(s.numKnots(), 1);
    EXPECT_EQ(s.getKnotPos(0), Eigen::Vector3d(1, 2, 3));
}

// --- The IEKF's recursive-control-point window (last 4 knots) must be
// --- unaffected by pruning.

TEST(SplinePrune, RcpRoundTripAfterPrune)
{
    SplineState full = makeSpline(100);
    SplineState pruned = makeSpline(100);
    pruned.pruneFrontKnots(40);

    EXPECT_EQ(full.getRCPs(), pruned.getRCPs());

    // updateRCPs writes the last 4 knots and re-chains their quaternions;
    // the chain root for local index 0 is q_idle[2] which pruning must have
    // slid correctly.
    Eigen::Matrix<double, 24, 1> cp = full.getRCPs();
    cp.segment<3>(0) += Eigen::Vector3d(0.01, -0.02, 0.005);
    cp.segment<3>(21) += Eigen::Vector3d(0.001, 0.002, -0.001);
    full.updateRCPs(cp);
    pruned.updateRCPs(cp);
    EXPECT_EQ(full.getRCPs(), pruned.getRCPs());
    for (int64_t t = pruned.maxTimeNs() - 5 * kDtNs; t <= pruned.maxTimeNs(); t += kDtNs / 4) {
        PoseSample a = sampleAt(full, t);
        PoseSample b = sampleAt(pruned, t);
        EXPECT_EQ(a.pos, b.pos);
        EXPECT_EQ(a.q.coeffs(), b.q.coeffs());
    }
}

// --- est_window protocol: a Mapping-style receiver reconstructing the spline
// --- from getSplineMsg windows must converge to the sender's trajectory,
// --- with pruning active on the sender, the receiver, or both.

namespace
{

// Mirrors Mapping::getEstCallback + process(): build a window spline from the
// message, then merge it into the receiver via updateKnots.
void applyWindow(SplineState& receiver, const estimate_msgs::msg::Spline& msg)
{
    SplineState w;
    w.init(msg.dt, 0, msg.start_t, static_cast<int>(msg.start_idx));
    for (const auto& k : msg.knots) {
        w.addOneStateKnot(Eigen::Vector3d(k.position.x, k.position.y, k.position.z),
                          Eigen::Vector3d(k.orientation_del.x, k.orientation_del.y,
                                          k.orientation_del.z));
    }
    Eigen::Quaterniond q0(msg.start_q.w, msg.start_q.x, msg.start_q.y, msg.start_q.z);
    for (int i = 0; i < 3 && i < static_cast<int>(msg.idles.size()); i++) {
        const auto& idle = msg.idles[i];
        w.setIdles(i, Eigen::Vector3d(idle.position.x, idle.position.y, idle.position.z),
                   Eigen::Vector3d(idle.orientation_del.x, idle.orientation_del.y,
                                   idle.orientation_del.z),
                   q0);
    }
    receiver.setTimeIntervalNs(w.getKnotTimeIntervalNs());
    receiver.updateKnots(&w);
}

}  // namespace

TEST(SplineMsgProtocol, ReceiverTracksSenderWithPruningBothSides)
{
    SplineState sender;
    sender.init(kDtNs, 0, kStartNs);
    SplineState receiver;
    receiver.init(1, 0, kStartNs);  // dt placeholder, as in Mapping startCallBack

    // RESPLE-style publish loop: grow a few knots, publish the window
    // (hint = previous published total - 1), prune both sides.
    int64_t published_total = 0;
    int added = 0;
    for (int cycle = 0; cycle < 120; cycle++) {
        for (int j = 0; j < 3; j++) {
            sender.addOneStateKnot(knotPos(added), knotOrtDel(added));
            added++;
        }
        if (sender.totalKnots() > published_total) {
            estimate_msgs::msg::Spline msg;
            sender.getSplineMsg(msg, std::max<int64_t>(published_total - 1, 0));
            published_total = sender.totalKnots();
            applyWindow(receiver, msg);
        }
        sender.pruneFrontKnots(50);
        receiver.pruneFrontKnots(80);
    }

    ASSERT_EQ(receiver.totalKnots(), sender.totalKnots());
    ASSERT_EQ(receiver.maxTimeNs(), sender.maxTimeNs());
    // The receiver's retained window is wider than the sender's; compare
    // trajectories over the intersection.
    const int64_t t0 = std::max(receiver.minTimeNs(), sender.minTimeNs());
    for (int64_t t = t0; t <= sender.maxTimeNs(); t += kDtNs) {
        PoseSample a = sampleAt(sender, t);
        PoseSample b = sampleAt(receiver, t);
        EXPECT_LT((a.pos - b.pos).norm(), 1e-12) << "t=" << t;
        EXPECT_LT(std::abs(std::abs(a.q.dot(b.q)) - 1.0), 1e-12) << "t=" << t;
    }
}

TEST(SplineMsgProtocol, UnprunedProtocolUnchanged)
{
    // Regression guard for the absolute-index rewrite: without pruning the
    // emitted windows must look exactly like the historical ones.
    SplineState sender;
    sender.init(kDtNs, 0, kStartNs);
    for (int i = 0; i < 12; i++) {
        sender.addOneStateKnot(knotPos(i), knotOrtDel(i));
    }
    estimate_msgs::msg::Spline msg;
    sender.getSplineMsg(msg, 0);
    EXPECT_EQ(static_cast<int64_t>(msg.start_idx), 0);  // first publish sends everything
    EXPECT_EQ(static_cast<int64_t>(msg.knots.size()), 12);
    EXPECT_EQ(msg.start_t, kStartNs + 7 * kDtNs);  // time of knot total-5
    EXPECT_EQ(msg.dt, kDtNs);
    ASSERT_EQ(msg.idles.size(), 3u);

    sender.addOneStateKnot(knotPos(12), knotOrtDel(12));
    estimate_msgs::msg::Spline msg2;
    sender.getSplineMsg(msg2, 12 - 1);
    // Hint 11 vs total-5=8: contiguity picks min(8, last_start_idx+1=8) = 8.
    EXPECT_EQ(static_cast<int64_t>(msg2.start_idx), 8);
    EXPECT_EQ(static_cast<int64_t>(msg2.knots.size()), 5);
}

TEST(SplineMsgProtocol, StaleWindowBeforeReceiverPruneIsIgnored)
{
    // A window that only covers knots the receiver has already pruned must be
    // a no-op (no OOB, no append at the wrong index).
    SplineState sender;
    sender.init(kDtNs, 0, kStartNs);
    SplineState receiver;
    receiver.init(1, 0, kStartNs);

    for (int i = 0; i < 40; i++) {
        sender.addOneStateKnot(knotPos(i), knotOrtDel(i));
    }
    estimate_msgs::msg::Spline early;
    sender.getSplineMsg(early, 0);  // covers absolute knots [0, 40)

    SplineState receiver_ref;
    receiver_ref.init(1, 0, kStartNs);
    applyWindow(receiver, early);
    applyWindow(receiver_ref, early);

    receiver.pruneFrontKnots(10);  // drops absolute knots [0, 30)
    const int64_t knots_before = receiver.numKnots();
    const int64_t total_before = receiver.totalKnots();

    // Re-deliver the stale window (e.g. a late-arriving duplicate).
    applyWindow(receiver, early);
    EXPECT_EQ(receiver.numKnots(), knots_before);
    EXPECT_EQ(receiver.totalKnots(), total_before);
    for (int64_t t = receiver.minTimeNs(); t <= receiver.maxTimeNs(); t += kDtNs / 2) {
        PoseSample a = sampleAt(receiver_ref, t);
        PoseSample b = sampleAt(receiver, t);
        EXPECT_EQ(a.pos, b.pos) << "t=" << t;
        EXPECT_EQ(a.q.coeffs(), b.q.coeffs()) << "t=" << t;
    }
}

// --- Bug A3 regression: orientation (J_q) and angular-velocity (J_w) Jacobians
// --- at the spline-START boundary. For idx_l < 2 the interpolation window has
// --- idle slots at the front (idx_window > 0, size_J < 4); the rotational
// --- Jacobian loops used to index window slot i instead of (4-size_J)+i, so
// --- they read idle-knot derivatives and misattributed the sensitivity by
// --- idx_window knots (and dropped the last real knot). Validate analytic vs
// --- central-difference numerical Jacobians there; the position Jacobian
// --- (always correct) is the harness control. The spline's quaternion factors
// --- are exactly unit, so the final normalize() is a no-op and the analytic
// --- Jacobian is exact.

namespace
{
// Synthetic spline with one knot's position OR ort-delta nudged on one axis,
// for central-difference numerical Jacobians.
SplineState makeSplineNudged(int N, int k, bool ort, int axis, double eps)
{
    SplineState s;
    s.init(kDtNs, 0, kStartNs);
    for (int i = 0; i < N; i++) {
        Eigen::Vector3d p = knotPos(i);
        Eigen::Vector3d od = knotOrtDel(i);
        (ort ? od : p)(axis) += (i == k) ? eps : 0.0;
        s.addOneStateKnot(p, od);
    }
    return s;
}
}  // namespace

TEST(SplineJacobianBoundary, RotationJacobiansMatchNumericalNearStart)
{
    const int N = 5;
    SplineState base = makeSpline(N);
    const double eps = 1e-5;
    // idx_l = 0 (idx_window 2, size_J 2) and idx_l = 1 (idx_window 1, size_J 3).
    // Steady state (idx_window 0, size_J 4) is the path left unchanged by the fix.
    const int64_t times[] = { kStartNs + kDtNs / 2, kStartNs + 3 * kDtNs / 2 };
    for (int64_t t : times) {
        Eigen::Vector3d p_base;
        Eigen::Quaterniond q_base, q_base2;
        Eigen::Vector3d w_base;
        Jacobian J_p;
        Jacobian43 J_q_quat, J_q_pose;
        Jacobian33 J_w;
        base.itpQuaternion(t, &q_base, &w_base, &J_q_quat, &J_w);
        base.itpPose(t, &p_base, &J_p, &q_base2, &J_q_pose);
        ASSERT_GT(J_q_quat.d_val_d_knot.size(), 0u);
        ASSERT_LT(J_q_quat.d_val_d_knot.size(), 4u) << "expected a boundary window (size_J<4) at t=" << t;
        const int idx0 = static_cast<int>(J_q_quat.start_idx);
        ASSERT_EQ(static_cast<int>(J_p.start_idx), idx0);
        ASSERT_EQ(J_q_pose.d_val_d_knot.size(), J_q_quat.d_val_d_knot.size());
        for (size_t e = 0; e < J_q_quat.d_val_d_knot.size(); e++) {
            const int k = idx0 + static_cast<int>(e);
            // itpPose and itpQuaternion must agree on the orientation Jacobian
            // (both carry the same fix).
            EXPECT_LT((J_q_pose.d_val_d_knot[e] - J_q_quat.d_val_d_knot[e]).norm(), 1e-9)
                << "itpPose vs itpQuaternion J_q disagree, t=" << t << " knot=" << k;
            for (int a = 0; a < 3; a++) {
                SplineState op = makeSplineNudged(N, k, true, a, +eps);
                SplineState om = makeSplineNudged(N, k, true, a, -eps);
                Eigen::Quaterniond qp, qm;
                Eigen::Vector3d wp, wm;
                op.itpQuaternion(t, &qp, &wp, nullptr, nullptr);
                om.itpQuaternion(t, &qm, &wm, nullptr, nullptr);
                Eigen::Vector4d dq_num;  // [w,x,y,z] to match the Jacobian rows
                dq_num << qp.w() - qm.w(), qp.x() - qm.x(), qp.y() - qm.y(), qp.z() - qm.z();
                dq_num /= (2 * eps);
                EXPECT_LT((dq_num - J_q_quat.d_val_d_knot[e].col(a)).norm(), 1e-4)
                    << "J_q t=" << t << " knot=" << k << " axis=" << a
                    << "\n  num=" << dq_num.transpose()
                    << "\n  ana=" << J_q_quat.d_val_d_knot[e].col(a).transpose();
                Eigen::Vector3d dw_num = (wp - wm) / (2 * eps);
                EXPECT_LT((dw_num - J_w.d_val_d_knot[e].col(a)).norm(), 1e-4)
                    << "J_w t=" << t << " knot=" << k << " axis=" << a
                    << "\n  num=" << dw_num.transpose()
                    << "\n  ana=" << J_w.d_val_d_knot[e].col(a).transpose();
                // Position control (J_p always correct): ∂p/∂(knot-pos axis a) =
                // blend-coeff · e_a. Linear, so central difference is exact.
                SplineState pp = makeSplineNudged(N, k, false, a, +eps);
                SplineState pm = makeSplineNudged(N, k, false, a, -eps);
                Eigen::Vector3d dp_num = (pp.itpPosition(t) - pm.itpPosition(t)) / (2 * eps);
                Eigen::Vector3d dp_ana = Eigen::Vector3d::Zero();
                dp_ana(a) = J_p.d_val_d_knot[e];
                EXPECT_LT((dp_num - dp_ana).norm(), 1e-9)
                    << "J_p control t=" << t << " knot=" << k << " axis=" << a;
            }
        }
    }
}

// --- Regression: itpQuaternion must write *q_out for EVERY non-null pointer
// --- combination. The (q_out, J_q=null, J_w!=null) combination — used by
// --- Estimator::getLastTwistCovariance — entered the Jacobian branch, whose
// --- only q_out write was gated on J_q, leaving the caller's (uninitialized)
// --- quaternion untouched.

TEST(SplineQuery, QOutWrittenForAllPointerCombinations)
{
    SplineState s = makeSpline(40);
    for (int64_t t : {s.minTimeNs(), s.minTimeNs() + 37 * kDtNs / 10,
                      (s.minTimeNs() + s.maxTimeNs()) / 2, s.maxTimeNs()}) {
        Eigen::Quaterniond q_ref;
        s.itpQuaternion(t, &q_ref);  // plain path: known-good

        // Poison sentinel: survives the call only if q_out is never written.
        const Eigen::Quaterniond poison(1e300, -3.14, 42.0, 7.7);

        Eigen::Vector3d w;
        Jacobian33 J_w;
        Eigen::Quaterniond q_jw = poison;
        s.itpQuaternion(t, &q_jw, &w, nullptr, &J_w);  // the regression combo
        ASSERT_NE(q_jw.w(), poison.w()) << "q_out not written (J_w-only path) t=" << t;
        EXPECT_LT(q_jw.angularDistance(q_ref), 1e-12) << "t=" << t;

        Jacobian43 J_q;
        Eigen::Quaterniond q_jq = poison;
        s.itpQuaternion(t, &q_jq, nullptr, &J_q);
        ASSERT_NE(q_jq.w(), poison.w()) << "q_out not written (J_q path) t=" << t;
        EXPECT_LT(q_jq.angularDistance(q_ref), 1e-12) << "t=" << t;

        Eigen::Quaterniond q_both = poison;
        Eigen::Vector3d w2;
        Jacobian43 J_q2;
        Jacobian33 J_w2;
        s.itpQuaternion(t, &q_both, &w2, &J_q2, &J_w2);
        ASSERT_NE(q_both.w(), poison.w()) << "q_out not written (both-J path) t=" << t;
        EXPECT_LT(q_both.angularDistance(q_ref), 1e-12) << "t=" << t;
    }
}

// --- Regression twin of the above for w_out: the (q_out, w_out, J_q!=null,
// --- J_w==null) combination entered the Jacobian branch, whose only w_out
// --- write was gated on J_w — leaving a requested angular velocity unwritten
// --- (caller-side garbage). No production caller passes this combo today, but
// --- the "write every non-null out-param" contract must hold for w_out too.

TEST(SplineQuery, WOutWrittenOnJQOnlyPath)
{
    SplineState s = makeSpline(40);
    for (int64_t t : {s.minTimeNs(), s.minTimeNs() + 37 * kDtNs / 10,
                      (s.minTimeNs() + s.maxTimeNs()) / 2, s.maxTimeNs()}) {
        Eigen::Vector3d w_ref;
        s.itpQuaternion(t, nullptr, &w_ref);  // plain path: known-good angular velocity

        const Eigen::Vector3d poison(1e300, 1e300, 1e300);
        Eigen::Quaterniond q = Eigen::Quaterniond(1e300, -3.14, 42.0, 7.7);
        Eigen::Vector3d w = poison;
        Jacobian43 J_q;
        s.itpQuaternion(t, &q, &w, &J_q, nullptr);  // J_q set, J_w null
        ASSERT_NE(w.x(), poison.x()) << "w_out not written (J_q-only path) t=" << t;
        EXPECT_LT((w - w_ref).norm(), 1e-12) << "w_out wrong (J_q-only path) t=" << t;
    }
}
