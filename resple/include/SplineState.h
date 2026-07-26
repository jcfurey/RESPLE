#pragma once

#include <iostream>
#include "utils/eigen_utils.hpp"
#include "utils/math_tools.h"
#include "estimate_msgs/msg/spline.hpp"
#include "estimate_msgs/msg/knot.hpp"

// Log an invariant violation at most once per (file, line) to avoid spamming.
// The default workspace build sets -DNDEBUG, which compiles out assert(),
// meaning the code after a failed assert would silently execute with invalid
// state (often an out-of-bounds vector access). These helpers replace the
// asserts with a log + early return / safe fallback.
#define RESPLE_LOG_INVARIANT_ONCE(msg)                                         \
    do {                                                                       \
        static bool _respleLogged = false;                                     \
        if (!_respleLogged) {                                                  \
            std::cerr << "[SplineState] invariant violated (" << __FILE__      \
                      << ":" << __LINE__ << "): " << msg << std::endl;         \
            _respleLogged = true;                                              \
        }                                                                      \
    } while (0)

template <class MatT>
struct JacobianStruct {
    size_t start_idx;
    std::vector<MatT> d_val_d_knot;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef JacobianStruct<double> Jacobian;
typedef JacobianStruct<Eigen::Matrix<double, 4, 3>> Jacobian43;
typedef JacobianStruct<Eigen::Matrix3d> Jacobian33;

class SplineState
{

  public:

    SplineState() {};

    void init(int64_t dt_ns_, int num_knot_, int64_t start_t_ns_, int start_i_ = 0,
              Eigen::Vector3d t0 = Eigen::Vector3d::Zero(), Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity())
    {
        if (dt_ns_ <= 0) {
            std::cerr << "[SplineState] init() called with dt_ns=" << dt_ns_ << " (must be > 0), using dt_ns=1 as placeholder\n";
            dt_ns_ = 1;
        }
        // Note: the historical `if_first` flag was set true here and never
        // set false anywhere, making the !if_first branch in minTimeNs() dead
        // code. The flag and its branch were removed; minTimeNs() now always
        // returns start_t_ns. If a future change needs to extend the queryable
        // range one knot before start_t_ns (the original intent), reintroduce
        // the flag with a clear flip site (e.g., after the first
        // addOneStateKnot call, or when num_knot exceeds the spline order).
        dt_ns = dt_ns_;
        start_t_ns = start_t_ns_;
        num_knot = num_knot_;
        num_knots_pruned_ = 0;
        update_gap_events_ = 0;
        last_start_idx_ = 0;
        inv_dt = 1e9 / dt_ns;
        start_i = start_i_;
        pow_inv_dt[0] = 1.0;
        pow_inv_dt[1] = inv_dt;
        pow_inv_dt[2] = inv_dt * inv_dt;
        pow_inv_dt[3] = pow_inv_dt[2] * inv_dt;
        t_idle = {t0, t0, t0};
        ort_delta_idle = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        q_idle = {q0, q0, q0};
        // A re-init (e.g. Mapping consuming a RESPLE respawn) must drop the
        // previous run's knots: num_knot restarts at num_knot_ (all callers
        // pass 0) while the deques would otherwise keep their old elements,
        // so knot index i would silently read the PREVIOUS run's data.
        t_knots.clear();
        q_knots.clear();
        ort_delta.clear();
    }

    void setTimeIntervalNs(int64_t t_ns)
    {
        dt_ns = t_ns;
        inv_dt = 1e9 / dt_ns;
        pow_inv_dt[0] = 1.0;
        pow_inv_dt[1] = inv_dt;
        pow_inv_dt[2] = inv_dt * inv_dt;
        pow_inv_dt[3] = pow_inv_dt[2] * inv_dt;
    }    

    Eigen::Matrix<double, 24, 1> getRCPs()
    {
        Eigen::Matrix<double, 24, 1> RCPs;
        // num_knot < 4 makes (num_knot - 4 + i) negative for i=0, OOB-reading
        // the deque. Today num_knot is monotonically growing (no pruning yet),
        // so steady-state safe — but Phase 3.1 sliding-window pruning would
        // make this fire. Defensive zero return matches the IEKF's behaviour
        // when state is unavailable; the IEKF then sees no change and the
        // numerical-failure counter eventually trips.
        if (num_knot < 4) {
            RESPLE_LOG_INVARIANT_ONCE("getRCPs called with num_knot=" << num_knot << " < 4; returning zero");
            RCPs.setZero();
            return RCPs;
        }
        for (int i = 0; i < 4; i++) {
            RCPs.block<3, 1>(i*6, 0) = t_knots[num_knot - 4 + i];
            RCPs.block<3, 1>(i*6+3, 0) = ort_delta[num_knot - 4 + i];
        }
        return RCPs;
    }

    void updateRCPs(const Eigen::Matrix<double, 24, 1>& cp)
    {
        if (num_knot < 4) {
            RESPLE_LOG_INVARIANT_ONCE("updateRCPs called with num_knot < 4 (need 4 active knots); skipping");
            return;
        }
        for (int i = 0; i < 4; i++) {
            t_knots[num_knot - 4 + i] = cp.block<3, 1>(i*6, 0);
            ort_delta[num_knot - 4 + i] = cp.block<3, 1>(i*6 + 3, 0);
            Eigen::Quaterniond q_del;
            Quater::exp(ort_delta[num_knot - 4 + i], q_del);
            if (int(num_knot) - 4 + i > 0) {
                q_knots[num_knot - 4 + i] = q_knots[num_knot - 5 + i] * q_del;
            } else {
                // int(num_knot) - 4 + i == 0 is the only remaining case given
                // the guard above; the previous "< 0" branch is unreachable.
                q_knots[num_knot - 4 + i] = q_idle[2] * q_del;
            }
        }
    }    

    void addOneStateKnot(Eigen::Vector3d pos, Eigen::Vector3d ort_del)
    {
        t_knots.push_back(pos);
        ort_delta.push_back(ort_del);
        Eigen::Quaterniond ort;
        Eigen::Quaterniond q_del;
        Quater::exp(ort_del, q_del);
        if (num_knot > 0) {
            ort = q_knots.back() * q_del;
        } else {
            ort = q_idle[2] * q_del;
        }
        q_knots.push_back(ort);
        num_knot++;
    }    

    // Receiver-side idle reconstruction (Mapping est_window ingest). Call in
    // order idx = 0, 1, 2. Chains under the same "delta ARRIVING at knot j-3"
    // convention pruneFrontKnots and prepareInterpolation use: q_idle[j] =
    // q_idle[j-1] * exp(ort_delta_idle[j]), and knot 0's quaternion chains
    // from q_idle[2] via knot 0's OWN delta (ort_delta[0]).
    //
    // The previous implementation chained each NEXT slot from the delta being
    // stored (q_idle[2] = q_idle[1]*exp(delta_of_slot_1), q_knots[0] =
    // q_idle[2]*exp(delta_of_slot_2)) — one index off the convention (bug-hunt
    // 2026-07-02 finding #25). Benign while every caller feeds zero deltas
    // (both conventions coincide at identity), but silently wrong the moment a
    // pruned sender's nonzero idles reach a fresh receiver.
    void setIdles(int idx, const Eigen::Vector3d& t, const Eigen::Vector3d& ort_del, const Eigen::Quaterniond& q_idle0)
    {
        if (idx < 0 || idx > 2) {
            RESPLE_LOG_INVARIANT_ONCE("setIdles called with idx=" << idx << " outside [0,2]");
            return;
        }
        t_idle[idx] = t;
        ort_delta_idle[idx] = ort_del;
        Eigen::Quaterniond q_del;
        Quater::exp(ort_del, q_del);
        if (idx == 0) {
            // Slot 0's arriving delta refers to the (unknown) knot before the
            // idle window; the anchor quaternion is supplied directly.
            q_idle[0] = q_idle0;
        } else {
            q_idle[idx] = q_idle[idx - 1] * q_del;
        }
        if (idx == 2) {
            if (num_knot == 0) {
                RESPLE_LOG_INVARIANT_ONCE("setIdles(idx=2) requires num_knot>0; skipping q_knots[0] write");
                return;
            }
            Eigen::Quaterniond q_del0;
            Quater::exp(ort_delta[0], q_del0);
            q_knots[0] = q_idle[2] * q_del0;
        }
    }

    void setOneStateKnot(int i, const Eigen::Vector3d& pos, const Eigen::Vector3d& ort_del)
    {
        // Guard against out-of-bounds knot index before touching any vector.
        // With -DNDEBUG the old assert was a no-op, so the t_knots[i] /
        // ort_delta[i] writes below were silently UB for i<0 or i>=num_knot.
        if (i < 0 || static_cast<int64_t>(i) >= num_knot) {
            RESPLE_LOG_INVARIANT_ONCE("setOneStateKnot i=" << i << " out of [0," << num_knot << ")");
            return;
        }
        t_knots[i] = pos;

        ort_delta[i] = ort_del;
        Eigen::Quaterniond q_del;
        Quater::exp(ort_del, q_del);
        if (i > 0) {
            q_knots[i] = q_knots[i - 1] * q_del;
        } else {
            // i == 0 is the only remaining case given the bounds guard above.
            q_knots[i] = q_idle[2] * q_del;
        }
    }

    void updateKnots(SplineState* other)
    {
        int64_t num_knots_other = other->numKnots();
        // The incoming idles describe the spline ORIGIN (knots -3..-1 of
        // absolute index 0); they are only valid here while our knot 0 is
        // still the absolute origin. After pruning, our idle slots hold the
        // slid pre-window knots and must not be clobbered.
        if (num_knots_pruned_ == 0 && num_knot <= num_knots_other) {
            q_idle = other->q_idle;
            ort_delta_idle = other->ort_delta_idle;
            t_idle = other->t_idle;
        }
        for (int64_t i = 0; i < num_knots_other; i++) {
            // other->start_i carries the sender's ABSOLUTE knot index (the
            // est_window protocol); translate to this spline's deque index,
            // which is offset by any front-pruning done locally (Phase 3.1).
            const int64_t target = i + other->start_i - num_knots_pruned_;
            if (target < 0) {
                // Window overlaps knots this spline already pruned — stale
                // content, nothing to update.
                continue;
            }
            if (target < num_knot) {
                setOneStateKnot(static_cast<int>(target), other->t_knots[i], other->ort_delta[i]);
            } else {
                if (target > num_knot) {
                    update_gap_events_++;
                    if (num_knot == 0) {
                        // Startup origin artifact, expected once per (re)start:
                        // windows published before the start-signal handshake
                        // completed are rejected by design, so the first
                        // applied window begins at a nonzero absolute index.
                        std::cerr << "[SplineState] updateKnots: first window begins at absolute idx "
                                  << target << " (startup origin; expected after (re)start)\n";
                        start_t_ns += (target - num_knot) * dt_ns;
                        num_knots_pruned_ += (target - num_knot);
                    } else {
                        if ((update_gap_events_ & (update_gap_events_ - 1)) == 0) {
                            // Mid-run gap = real est_window loss (the Mapping
                            // ingest queue should prevent this). Log counts at
                            // powers of two: early visibility without spam.
                            std::cerr << "[SplineState] updateKnots gap #" << update_gap_events_
                                      << ": target=" << target << " > num_knot=" << num_knot
                                      << " (missed est_window); padding " << (target - num_knot)
                                      << " knot(s) to keep the time axis aligned\n";
                        }
                        // Re-align the time axis (bug-hunt 2026-07-02 finding
                        // #15): appending the window sequentially at num_knot
                        // would shift EVERY subsequent knot's time by the gap
                        // width, permanently skewing the replica. Pad the
                        // missing indices with constant-pose knots (repeat the
                        // last position, zero orientation delta) so this
                        // window's knots land at their true absolute indices —
                        // the padded span is wrong (its data was lost) but the
                        // error stays bounded to the gap itself.
                        while (num_knot < target) {
                            addOneStateKnot(t_knots.back(), Eigen::Vector3d::Zero());
                        }
                    }
                }
                addOneStateKnot(other->t_knots[i], other->ort_delta[i]);
            }
        }
    }

    // Phase 3.1 (HARDENING.md): sliding-window knot pruning.
    //
    // Drops the oldest knots so at most `keep_knots` remain in the deques and
    // slides the 3-slot idle window forward to take their place. The idle
    // knots act as the knots at indices -3..-1 relative to knot 0 (t_idle[j] /
    // q_idle[j] hold the pose of knot j-3, ort_delta_idle[j] the delta
    // arriving at it — the same convention prepareInterpolation / itpPose /
    // itpQuaternion read them with), so after pruning K knots the old knots
    // [K-3 .. K-1] become the new idles and interpolation over the RETAINED
    // time range is bit-for-bit identical to the unpruned spline.
    //
    // minTimeNs() advances by K*dt_ns. Absolute knot indexing — totalKnots()
    // and the getSplineMsg start_idx contract with the Mapping node — is
    // preserved via num_knots_pruned_. Returns the number of knots pruned.
    int64_t pruneFrontKnots(int64_t keep_knots)
    {
        // Hard floor: the IEKF's recursive window reads the last 4 knots
        // (getRCPs/updateRCPs) and the est_window message the last 5; keep a
        // margin beyond both so a misconfigured caller cannot starve them.
        keep_knots = std::max<int64_t>(keep_knots, 8);
        const int64_t prune = num_knot - keep_knots;
        if (prune <= 0) {
            return 0;
        }
        // New idle slot j holds the knot at (new) index j-3 == old index
        // prune-3+j: an old regular knot when that is >= 0, otherwise the old
        // idle slot it maps to (only reachable for prune < 3).
        std::array<Eigen::Vector3d, 3> t_idle_new;
        std::array<Eigen::Vector3d, 3> ort_idle_new;
        std::array<Eigen::Quaterniond, 3> q_idle_new;
        for (int j = 0; j < 3; j++) {
            const int64_t src = prune - 3 + j;
            if (src >= 0) {
                t_idle_new[j] = t_knots[src];
                ort_idle_new[j] = ort_delta[src];
                q_idle_new[j] = q_knots[src];
            } else {
                t_idle_new[j] = t_idle[3 + src];
                ort_idle_new[j] = ort_delta_idle[3 + src];
                q_idle_new[j] = q_idle[3 + src];
            }
        }
        for (int j = 0; j < 3; j++) {
            t_idle[j] = t_idle_new[j];
            ort_delta_idle[j] = ort_idle_new[j];
            q_idle[j] = q_idle_new[j];
        }
        t_knots.erase(t_knots.begin(), t_knots.begin() + prune);
        q_knots.erase(q_knots.begin(), q_knots.begin() + prune);
        ort_delta.erase(ort_delta.begin(), ort_delta.begin() + prune);
        num_knot -= prune;
        num_knots_pruned_ += prune;
        start_t_ns += prune * dt_ns;
        return prune;
    }

    int64_t getKnotTimeNs(size_t i) const
    {
        return start_t_ns + i * dt_ns;
    }

    int64_t getKnotTimeIntervalNs() const
    {
        return dt_ns;
    }

    Eigen::Vector3d getKnotPos(size_t i) const
    {
        return t_knots[i];
    }

    Eigen::Quaterniond getKnotOrt(size_t i) const
    {
        return q_knots.at(i);
    }    

    Eigen::Vector3d getKnotOrtDel(size_t i) const
    {
        return ort_delta.at(i);
    }       

    int64_t maxTimeNs() const
    {
        if (num_knot == 1) {
           return start_t_ns;
        }
        return start_t_ns + (num_knot - 1) * dt_ns - 1;
    }

    int64_t minTimeNs() const
    {
        // Was: start_t_ns + dt_ns * (!if_first ? -1 : 0)
        // The `if_first` flag was always true (set in init(), never cleared
        // anywhere). The negative-offset branch was dead. See init() for the
        // rationale and the path to restore the original-intent behaviour.
        return start_t_ns;
    }

    int64_t numKnots() const
    {
        return num_knot;
    }

    // Knots currently held PLUS knots dropped by pruneFrontKnots — i.e. the
    // monotonic count addOneStateKnot has produced since init(). This is the
    // index space the est_window protocol (getSplineMsg start_idx /
    // updateKnots start_i) is expressed in.
    int64_t totalKnots() const
    {
        return num_knot + num_knots_pruned_;
    }

    int64_t numKnotsPruned() const
    {
        return num_knots_pruned_;
    }

    template <int Derivative = 0>
    Eigen::Vector3d itpPosition (int64_t time_ns, Jacobian* J = nullptr) const
    {
        return itpEuclidean<Eigen::Vector3d, Derivative>(time_ns, t_idle, t_knots, J);
    }

    void itpQuaternion(int64_t t_ns, Eigen::Quaterniond* q_out = nullptr,
                       Eigen::Vector3d* w_out = nullptr, Jacobian43* J_q = nullptr, Jacobian33* J_w = nullptr) const
    {
        double u;
        int64_t idx0;
        int idx_r;
        std::array<Eigen::Vector3d, 4> t_delta;
        prepareInterpolation(t_ns, ort_delta_idle, ort_delta, idx0, u, t_delta, idx_r);
        Eigen::Vector4d p;
        Eigen::Vector4d coeff;
        baseCoeffsWithTime<0>(p, u);
        coeff = cumulative_blending_matrix * p;

        Eigen::Quaterniond cp0;
        if (idx_r > 3) {
            cp0 = q_knots[idx0-1];
        } else if (idx_r == 3) {
            cp0 = q_idle[2];
        } else if (idx_r == 2) {
            cp0 = q_idle[1];
        } else if (idx_r == 1) {
            cp0 = q_idle[0];
        } else {
            // idx_r < 1 means prepareInterpolation returned a window before
            // the first idle knot — caller queried outside the spline support.
            // Fall back to identity and short-circuit so callers don't act on
            // uninitialized q_out / w_out. (Previously an NDEBUG no-op assert.)
            RESPLE_LOG_INVARIANT_ONCE("itpQuaternion idx_r=" << idx_r << " out of range; returning identity");
            if (q_out) *q_out = Eigen::Quaterniond::Identity();
            if (w_out) *w_out = Eigen::Vector3d::Zero();
            if (J_q) { J_q->d_val_d_knot.clear(); J_q->start_idx = 0; }
            if (J_w) { J_w->d_val_d_knot.clear(); J_w->start_idx = 0; }
            return;
        }

        Eigen::Vector3d t_delta_scale[4];
        Eigen::Quaterniond q_delta_scale[4];
        Eigen::Quaterniond q_itps[4];
        Eigen::Vector3d w_itps[4];
        Eigen::Vector4d dcoeff;
        if (J_q || J_w) {
            Eigen::Matrix<double, 4, 3> dexp_dt[4];
            t_delta_scale[0] = t_delta[0] * coeff[0];
            t_delta_scale[1] = t_delta[1] * coeff[1];
            t_delta_scale[2] = t_delta[2] * coeff[2];
            t_delta_scale[3] = t_delta[3] * coeff[3];
            Quater::dexp(t_delta_scale[0], q_delta_scale[0], dexp_dt[0]);
            Quater::dexp(t_delta_scale[1], q_delta_scale[1], dexp_dt[1]);
            Quater::dexp(t_delta_scale[2], q_delta_scale[2], dexp_dt[2]);
            Quater::dexp(t_delta_scale[3], q_delta_scale[3], dexp_dt[3]);
            int size_J = std::min(idx_r + 1, 4);
            Eigen::Quaterniond q_r_all[4];
            q_r_all[3] = Eigen::Quaterniond::Identity();
            for (int i = 2; i >= 0; i-- ) {
                q_r_all[i] = q_delta_scale[i+1] * q_r_all[i+1];
            }
            if (J_q) {
                q_itps[0] = cp0 * q_delta_scale[0];
                q_itps[1] = q_itps[0] * q_delta_scale[1];
                q_itps[2] = q_itps[1] * q_delta_scale[2];
                q_itps[3] = q_itps[2] * q_delta_scale[3];
                q_itps[3].normalize();
                *q_out = q_itps[3];
                Eigen::Matrix4d Q_l_all[4];
                Quater::Qleft(cp0, Q_l_all[0]);
                Quater::Qleft(q_itps[0], Q_l_all[1]);
                Quater::Qleft(q_itps[1], Q_l_all[2]);
                Quater::Qleft(q_itps[2], Q_l_all[3]);
                J_q->d_val_d_knot.resize(size_J);
                J_q->start_idx = idx0;
                // Bug A3: map output entry i -> window SLOT (4-size_J)+i (= idx_window+i),
                // the slot holding deque knot idx0+i — mirroring the position Jacobian
                // (itpEuclidean/itpPose position use coeff[4-size_J+i]). Indexing the slot
                // by i directly is correct only when size_J==4; for size_J<4 (a query in the
                // first two knot-intervals of the spline start) it read idle-knot
                // derivatives and misattributed the orientation sensitivity by idx_window
                // knots, dropping the last real knot's contribution.
                for (int i = size_J - 1; i >= 0; i--) {
                    const int s = (4 - size_J) + i;
                    Eigen::Matrix4d Q_r_all;
                    Quater::Qright(q_r_all[s], Q_r_all);
                    J_q->d_val_d_knot[i].noalias() = coeff[s] * Q_r_all * Q_l_all[s] * dexp_dt[s];
                }
            } else if (q_out) {
                // q_out requested alongside J_w only (e.g. the twist-covariance
                // path): the write above is gated on J_q, so compose the
                // interpolated quaternion here from the q_delta_scale chain
                // dexp already produced. Without this, *q_out kept its
                // caller-side value — a default-constructed Eigen::Quaterniond
                // is UNINITIALIZED, and getLastTwistCovariance rotated the
                // velocity covariance by stack garbage.
                Eigen::Quaterniond q_itp = cp0 * q_delta_scale[0] * q_delta_scale[1]
                                               * q_delta_scale[2] * q_delta_scale[3];
                q_itp.normalize();
                *q_out = q_itp;
            }
            if (J_w || w_out) {
                // ONE copy of the angular-velocity recursion serves both the
                // Jacobian path and a value-only w_out request (the previous
                // structure duplicated it and, worse, the J_w path wrote
                // *w_out unguarded — a null deref for a J_w-without-w_out
                // caller). "Write every non-null out-param" (2026-07-02
                // contract) holds for w_out as it does for q_out.
                baseCoeffsWithTime<1>(p, u);
                dcoeff = inv_dt * cumulative_blending_matrix * p;
                w_itps[0].setZero();
                w_itps[1] = 2 * dcoeff[1] * t_delta[1];
                w_itps[2] = q_delta_scale[2].inverse() * w_itps[1] + 2 * dcoeff[2] * t_delta[2];
                w_itps[3] = q_delta_scale[3].inverse() * w_itps[2] + 2 * dcoeff[3] * t_delta[3];
                if (w_out) {
                    *w_out = w_itps[3];
                }
            }
            if (J_w) {
                Eigen::Matrix<double, 3, 4> drot_dq[2];
                Quater::drot(w_itps[1], q_delta_scale[2], drot_dq[0]);
                Quater::drot(w_itps[2], q_delta_scale[3], drot_dq[1]);
                J_w->d_val_d_knot.resize(size_J);
                J_w->start_idx = idx0;
                // Bug A3: angular velocity depends on slots 1..3 only (slot 0 has
                // zero derivative). Compute the per-SLOT derivatives, then map
                // output entry i -> slot (4-size_J)+i (the slot holding deque knot
                // idx0+i), mirroring the J_q / position fix. The old hard-coded
                // d_val_d_knot[k]=slot-k assignment was correct only for size_J==4;
                // for size_J<4 it shifted the attribution by idx_window knots.
                Eigen::Matrix3d dw_dslot[4];
                dw_dslot[0].setZero();
                dw_dslot[1] = 2 * dcoeff[1] * q_delta_scale[3].inverse().toRotationMatrix() * q_delta_scale[2].inverse().toRotationMatrix();
                {
                    Eigen::Matrix3d tmp = coeff[2] * drot_dq[0] * dexp_dt[2];
                    dw_dslot[2] = q_delta_scale[3].inverse().toRotationMatrix() * (tmp + 2 * dcoeff[2] * Eigen::Matrix3d::Identity());
                }
                dw_dslot[3] = coeff[3] * drot_dq[1] * dexp_dt[3] + 2 * dcoeff[3] * Eigen::Matrix3d::Identity();
                for (int i = 0; i < size_J; i++) {
                    J_w->d_val_d_knot[i] = dw_dslot[(4 - size_J) + i];
                }
            }
        } else {
            t_delta_scale[0] = t_delta[0] * coeff[0];
            t_delta_scale[1] = t_delta[1] * coeff[1];
            t_delta_scale[2] = t_delta[2] * coeff[2];
            t_delta_scale[3] = t_delta[3] * coeff[3];
            Quater::exp(t_delta_scale[0], q_delta_scale[0]);
            Quater::exp(t_delta_scale[1], q_delta_scale[1]);
            Quater::exp(t_delta_scale[2], q_delta_scale[2]);
            Quater::exp(t_delta_scale[3], q_delta_scale[3]);
            if (q_out) {
                q_itps[0] = cp0 * q_delta_scale[0];
                q_itps[1] = q_itps[0] * q_delta_scale[1];
                q_itps[2] = q_itps[1] * q_delta_scale[2];
                q_itps[3] = q_itps[2] * q_delta_scale[3];
                q_itps[3].normalize();
                *q_out = q_itps[3];
            }
            if (w_out) {
                baseCoeffsWithTime<1>(p, u);
                dcoeff = inv_dt * cumulative_blending_matrix * p;

                w_itps[0].setZero();
                w_itps[1] = 2 * dcoeff[1] * t_delta[1];
                w_itps[2] = q_delta_scale[2].inverse() * w_itps[1] + 2 * dcoeff[2] * t_delta[2];
                w_itps[3] = q_delta_scale[3].inverse() * w_itps[2] + 2 * dcoeff[3] * t_delta[3];
                *w_out = w_itps[3];
            }
        }
    }  

    // Combined position + quaternion interpolation at the same time.
    // Shares prepareInterpolation and baseCoeffsWithTime work between the two.
    // Used in prepLiDAR's hot path (one call per measurement per IEKF iteration).
    void itpPose(int64_t t_ns,
                 Eigen::Vector3d* p_out, Jacobian* J_p,
                 Eigen::Quaterniond* q_out, Jacobian43* J_q) const
    {
        // Defensive clamp: callers should not query outside [minTimeNs, maxTimeNs],
        // but if t_ns overshoots maxTimeNs() then idx_l > num_knot-2, and
        // t_knots[idx0 + i] / ort_delta[idx0 + i] / q_knots[idx0-1] below
        // OOB-read the deque → SIGSEGV. Clamping here yields a degraded but
        // numerically defined boundary evaluation. Association::pointBodyToWorld
        // is the primary upstream clamp; this one is belt-and-braces.
        const int64_t t_min = minTimeNs();
        const int64_t t_max = maxTimeNs();
        if (t_ns < t_min || t_ns > t_max) {
            RESPLE_LOG_INVARIANT_ONCE("itpPose t_ns=" << t_ns << " outside ["
                << t_min << "," << t_max << "] (num_knot=" << num_knot
                << "); clamping to spline range");
            t_ns = std::max(t_min, std::min(t_max, t_ns));
        }
        int64_t t_ns_rel = t_ns - start_t_ns;
        int idx_l = floor(double(t_ns_rel) / double(dt_ns));
        int idx_r = idx_l + 1;
        int64_t idx0 = std::max(idx_l - 2, 0);
        double u = (t_ns - start_t_ns - idx_l * dt_ns) / double(dt_ns);

        Eigen::Vector4d p;
        baseCoeffsWithTime<0>(p, u);
        const int size_J = std::min(idx_r + 1, 4);
        const int idx_window = std::max(0, 2 - idx_l);
        // Cap n_active against the deque size (mirrors prepareInterpolation's
        // n_active_safe). The entry clamp already bounds t_ns, but this defends
        // the t_knots / ort_delta reads below if a future knot-prune shrinks the
        // deque under a stale idx0. All three knot deques have size == num_knot.
        const int n_active = std::min<int>(std::min(idx_l + 2, 4),
            std::max<int>(0, static_cast<int>(t_knots.size()) - static_cast<int>(idx0)));

        // Position interpolation
        if (p_out || J_p) {
            std::array<Eigen::Vector3d, 4> cps;
            for (int i = 0; i < 2 - idx_l; i++) cps[i] = t_idle[i + idx_l + 1];
            for (int i = 0; i < n_active; i++) cps[i + idx_window] = t_knots[idx0 + i];

            Eigen::Vector4d coeff_p = blending_matrix * p;
            if (p_out) {
                *p_out = coeff_p[0]*cps[0] + coeff_p[1]*cps[1] + coeff_p[2]*cps[2] + coeff_p[3]*cps[3];
            }
            if (J_p) {
                J_p->d_val_d_knot.resize(size_J);
                for (int i = 0; i < size_J; i++) J_p->d_val_d_knot[i] = coeff_p[4 - size_J + i];
                J_p->start_idx = idx0;
            }
        }

        // Quaternion interpolation (shares u, p, idx0, idx_r with above)
        if (q_out || J_q) {
            std::array<Eigen::Vector3d, 4> t_delta;
            for (int i = 0; i < 2 - idx_l; i++) t_delta[i] = ort_delta_idle[i + idx_l + 1];
            for (int i = 0; i < n_active; i++) t_delta[i + idx_window] = ort_delta[idx0 + i];

            Eigen::Vector4d coeff = cumulative_blending_matrix * p;

            Eigen::Quaterniond cp0;
            if (idx_r > 3) {
                // Guard the q_knots[idx0-1] read (same deque-shrink rationale as
                // the n_active cap above): fall back to identity if out of range.
                const int64_t cp0_idx = idx0 - 1;
                if (cp0_idx < 0 || cp0_idx >= static_cast<int64_t>(q_knots.size())) {
                    RESPLE_LOG_INVARIANT_ONCE("itpPose q_knots[idx0-1] idx=" << cp0_idx
                        << " out of range; returning identity quaternion");
                    if (q_out) *q_out = Eigen::Quaterniond::Identity();
                    if (J_q) { J_q->d_val_d_knot.clear(); J_q->start_idx = 0; }
                    return;
                }
                cp0 = q_knots[cp0_idx];
            }
            else if (idx_r == 3) cp0 = q_idle[2];
            else if (idx_r == 2) cp0 = q_idle[1];
            else if (idx_r == 1) cp0 = q_idle[0];
            else {
                // Query before the first idle knot — return identity and zero
                // Jacobian rather than leaving q_out / J_q garbage.
                RESPLE_LOG_INVARIANT_ONCE("itpPose idx_r=" << idx_r << " out of range; returning identity quaternion");
                if (q_out) *q_out = Eigen::Quaterniond::Identity();
                if (J_q) { J_q->d_val_d_knot.clear(); J_q->start_idx = 0; }
                return;
            }

            Eigen::Vector3d t_delta_scale[4];
            Eigen::Quaterniond q_delta_scale[4];
            t_delta_scale[0] = t_delta[0] * coeff[0];
            t_delta_scale[1] = t_delta[1] * coeff[1];
            t_delta_scale[2] = t_delta[2] * coeff[2];
            t_delta_scale[3] = t_delta[3] * coeff[3];

            if (J_q) {
                Eigen::Matrix<double, 4, 3> dexp_dt[4];
                Quater::dexp(t_delta_scale[0], q_delta_scale[0], dexp_dt[0]);
                Quater::dexp(t_delta_scale[1], q_delta_scale[1], dexp_dt[1]);
                Quater::dexp(t_delta_scale[2], q_delta_scale[2], dexp_dt[2]);
                Quater::dexp(t_delta_scale[3], q_delta_scale[3], dexp_dt[3]);
                Eigen::Quaterniond q_r_all[4];
                q_r_all[3] = Eigen::Quaterniond::Identity();
                for (int i = 2; i >= 0; i--) q_r_all[i] = q_delta_scale[i+1] * q_r_all[i+1];

                Eigen::Quaterniond q_itps[4];
                q_itps[0] = cp0 * q_delta_scale[0];
                q_itps[1] = q_itps[0] * q_delta_scale[1];
                q_itps[2] = q_itps[1] * q_delta_scale[2];
                q_itps[3] = q_itps[2] * q_delta_scale[3];
                q_itps[3].normalize();
                if (q_out) *q_out = q_itps[3];

                Eigen::Matrix4d Q_l_all[4];
                Quater::Qleft(cp0, Q_l_all[0]);
                Quater::Qleft(q_itps[0], Q_l_all[1]);
                Quater::Qleft(q_itps[1], Q_l_all[2]);
                Quater::Qleft(q_itps[2], Q_l_all[3]);
                J_q->d_val_d_knot.resize(size_J);
                J_q->start_idx = idx0;
                // Bug A3: map output entry i -> window SLOT (4-size_J)+i (= idx_window+i),
                // the slot holding deque knot idx0+i — mirroring the position Jacobian
                // (itpEuclidean/itpPose position use coeff[4-size_J+i]). Indexing the slot
                // by i directly is correct only when size_J==4; for size_J<4 (a query in the
                // first two knot-intervals of the spline start) it read idle-knot
                // derivatives and misattributed the orientation sensitivity by idx_window
                // knots, dropping the last real knot's contribution.
                for (int i = size_J - 1; i >= 0; i--) {
                    const int s = (4 - size_J) + i;
                    Eigen::Matrix4d Q_r_all;
                    Quater::Qright(q_r_all[s], Q_r_all);
                    J_q->d_val_d_knot[i].noalias() = coeff[s] * Q_r_all * Q_l_all[s] * dexp_dt[s];
                }
            } else {
                Quater::exp(t_delta_scale[0], q_delta_scale[0]);
                Quater::exp(t_delta_scale[1], q_delta_scale[1]);
                Quater::exp(t_delta_scale[2], q_delta_scale[2]);
                Quater::exp(t_delta_scale[3], q_delta_scale[3]);
                Eigen::Quaterniond q_itp = cp0 * q_delta_scale[0] * q_delta_scale[1]
                                              * q_delta_scale[2] * q_delta_scale[3];
                q_itp.normalize();
                *q_out = q_itp;
            }
        }
    }

    void getSplineMsg(estimate_msgs::msg::Spline& spline_msg, const int64_t start_idx_hint)
    {
        spline_msg.dt = dt_ns;
        // All indices in the message are ABSOLUTE (totalKnots() space) so the
        // receiver's window bookkeeping survives sender-side pruning; the
        // deque is addressed via (absolute - num_knots_pruned_).
        const int64_t total = totalKnots();
        int64_t sidx = std::min(std::max<int64_t>(total - 5, 0), start_idx_hint);
        int64_t start_idx = std::min(sidx, last_start_idx_ + 1);
        if (start_idx < num_knots_pruned_) {
            // The publish-then-prune ordering (and the >=8 retention floor)
            // keeps published windows ahead of pruning; clamp defensively so
            // the loop below cannot index before the first retained knot.
            RESPLE_LOG_INVARIANT_ONCE("getSplineMsg start_idx=" << start_idx
                << " precedes first retained knot " << num_knots_pruned_
                << "; clamping (window content lost to pruning)");
            start_idx = num_knots_pruned_;
        }
        spline_msg.start_idx = start_idx;
        // Time of absolute knot total-5 == local knot num_knot-5 (guarded for
        // the first cycles, where num_knot can still be < 5).
        //
        // CAUTION (2026-07-07 review): start_t describes knot total-5, but
        // start_idx above can be clamped LOWER (last_start_idx_+1 lags, or the
        // prune floor) — so start_t is NOT necessarily the time of the first
        // knot in this message. No current consumer relies on that pairing
        // (Mapping's updateKnots uses start_i + the receiver's own time axis;
        // window_st_ns is unused), but do not introduce one without fixing
        // this to getKnotTimeNs(start_idx - num_knots_pruned_).
        spline_msg.start_t = getKnotTimeNs(std::max<int64_t>(num_knot - 5, 0));
        for (int64_t i = start_idx - num_knots_pruned_; i < num_knot; i++) {
            estimate_msgs::msg::Knot knot_msg;
            knot_msg.position.x = t_knots[i].x();
            knot_msg.position.y = t_knots[i].y();
            knot_msg.position.z = t_knots[i].z();
            knot_msg.orientation_del.x = ort_delta[i].x();
            knot_msg.orientation_del.y = ort_delta[i].y();
            knot_msg.orientation_del.z = ort_delta[i].z();
            spline_msg.knots.push_back(knot_msg);
        }
        // The ANCHOR the receiver needs is the three knots immediately PRECEDING
        // start_idx (absolute start_idx-3 .. start_idx-1), not our own idle
        // slots. A fresh receiver rebuilds its idle slots from these and then
        // chains every knot quaternion off q_idle[2]
        // (addOneStateKnot: q_knots[0] = q_idle[2] * exp(ort_delta[0])), so
        // q_idle[2] must be the absolute orientation of knot start_idx-1.
        //
        // Our own idle slots describe knots (num_knots_pruned_-3 ..
        // num_knots_pruned_-1). Those coincide with the anchor only while
        // start_idx == num_knots_pruned_. At STARTUP we are unpruned (pruned=0)
        // while start_idx has walked up to the handshake cost T — every window
        // published before the receiver's start_time handshake completes is
        // rejected by it, and the last_start_idx_+1 throttle above advances
        // start_idx one knot per publish meanwhile. Shipping the origin as the
        // anchor then left the replica short of the rotation accumulated over
        // the skipped knots 0..T-1: because a window carries absolute POSITIONS
        // but only orientation DELTAS, the position error decays after two knot
        // intervals while the orientation error is PERMANENT (every later knot
        // chains from the mis-anchored knot 0) — the whole replica map/path/odom
        // stayed rotated. Measured 28.6 deg for a 0.5 s handshake at 0.57
        // deg/knot; see test/test_est_window_anchor.cpp (2026-07-26 bug hunt).
        //
        // Absolute index -> source, PER SLOT. Slot i must describe absolute knot
        // anchor_first+i; that is a retained knot when it is >= num_knots_pruned_
        // and one of our own idle slots otherwise. Our idle slot j describes
        // absolute knot (num_knots_pruned_-3+j), so the mapping is
        // j = (absolute - num_knots_pruned_) + 3.
        //
        // Do NOT collapse this to an all-or-nothing choice: falling back to
        // idle slot i wholesale is correct only when start_idx == pruned
        // exactly. For start_idx == pruned+1 or pruned+2 (the common SHORT
        // handshake blackout — Mapping's uninitialized spin is one 100 ms
        // worker iteration while RESPLE publishes ~30-40 windows/s, so
        // start_idx lands at 1..2) it is off by 1-2 knots, which is a
        // permanent 0.57 deg / 1.15 deg replica attitude offset at
        // 0.573 deg/knot. The clamp above guarantees start_idx >=
        // num_knots_pruned_, so j is always within [0,2] on the idle branch and
        // the deque index is always within [0,num_knot) on the knot branch.
        const int64_t anchor_first = start_idx - 3;  // absolute
        const auto anchorPos = [&](int64_t abs_idx) -> const Eigen::Vector3d& {
            const int64_t deq = abs_idx - num_knots_pruned_;
            return (deq >= 0) ? t_knots[deq] : t_idle[deq + 3];
        };
        const auto anchorDel = [&](int64_t abs_idx) -> const Eigen::Vector3d& {
            const int64_t deq = abs_idx - num_knots_pruned_;
            return (deq >= 0) ? ort_delta[deq] : ort_delta_idle[deq + 3];
        };
        for (int i = 0; i < 3; i++) {
            estimate_msgs::msg::Knot idle_msg;
            const Eigen::Vector3d& pos = anchorPos(anchor_first + i);
            const Eigen::Vector3d& del = anchorDel(anchor_first + i);
            idle_msg.position.x = pos.x();
            idle_msg.position.y = pos.y();
            idle_msg.position.z = pos.z();
            idle_msg.orientation_del.x = del.x();
            idle_msg.orientation_del.y = del.y();
            idle_msg.orientation_del.z = del.z();
            spline_msg.idles.push_back(idle_msg);
        }
        // start_q anchors slot 0 directly (setIdles(0) ignores its delta), so it
        // must be the ABSOLUTE orientation of knot anchor_first; slots 1 and 2
        // then chain forward through their own deltas to reach knot start_idx-1.
        const int64_t q_deq = anchor_first - num_knots_pruned_;
        const Eigen::Quaterniond& q_anchor =
            (q_deq >= 0) ? q_knots[q_deq] : q_idle[q_deq + 3];
        spline_msg.start_q.w = q_anchor.w();
        spline_msg.start_q.x = q_anchor.x();
        spline_msg.start_q.y = q_anchor.y();
        spline_msg.start_q.z = q_anchor.z();
        if (total < 5) {
            spline_msg.start_idx = 0;

        }
        last_start_idx_ = std::max<int64_t>(spline_msg.start_idx, total - 5);

    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    static const Eigen::Matrix4d blending_matrix;
    static const Eigen::Matrix4d base_coefficients;
    static const Eigen::Matrix4d cumulative_blending_matrix;

    int64_t dt_ns;
    double inv_dt;
    std::array<double, 4> pow_inv_dt;
    int64_t num_knot = 0;  // 0 until init(); a default-constructed spline reports no knots
    // Knots dropped from the FRONT of the deques by pruneFrontKnots (Phase
    // 3.1). Deque index = absolute index - num_knots_pruned_; start_t_ns is
    // advanced in lock-step so time-based lookups need no translation.
    int64_t num_knots_pruned_ = 0;
    // updateKnots index-gap events (first one per (re)start is the expected
    // startup origin artifact; mid-run events are real est_window loss).
    int64_t update_gap_events_ = 0;
    int64_t start_i;
    int64_t start_t_ns;
    int64_t last_start_idx_ = 0;

    std::array<Eigen::Vector3d, 3> t_idle;
    std::array<Eigen::Vector3d, 3> ort_delta_idle;
    Eigen::aligned_deque<Eigen::Quaterniond> q_idle;

    Eigen::aligned_deque<Eigen::Vector3d> t_knots;
    Eigen::aligned_deque<Eigen::Quaterniond> q_knots;
    Eigen::aligned_deque<Eigen::Vector3d> ort_delta;

    template <typename _KnotT, int Derivative = 0>
    _KnotT itpEuclidean(int64_t t_ns, const std::array<_KnotT, 3>& knots_idle,
                        const Eigen::aligned_deque<_KnotT>& knots, Jacobian* J = nullptr) const
    {
        double u;
        int64_t idx0;
        int idx_r;
        std::array<_KnotT,4> cps;
        prepareInterpolation(t_ns, knots_idle, knots, idx0, u, cps, idx_r);
        Eigen::Vector4d p, coeff;
        baseCoeffsWithTime<Derivative>(p, u);
        coeff = pow_inv_dt[Derivative] * (blending_matrix * p);
        _KnotT res_out = coeff[0] * cps.at(0) + coeff[1] * cps.at(1) + coeff[2] * cps.at(2) + coeff[3] * cps.at(3);
        if (J) {
            int size_J = std::min(idx_r + 1, 4);
            J->d_val_d_knot.resize(size_J);
            for (int i = 0; i < size_J; i++) {
                J->d_val_d_knot[i] = coeff[4 - size_J + i];
            }
            J->start_idx = idx0;
        }
        return res_out;
    }  

    template<typename _KnotT>
    void prepareInterpolation(int64_t t_ns, const std::array<_KnotT, 3>& knots_idle,
                              const Eigen::aligned_deque<_KnotT>& knots, int64_t& idx0, double& u,
                              std::array<_KnotT,4>& cps, int& idx_r) const
    {
        // Defensive clamp (mirrors itpPose). Without this, an out-of-range
        // t_ns lets idx_l + 2 exceed num_knot, OOB-reading the deque at
        // knots[idx0 + i]. Clamping yields a boundary evaluation instead.
        const int64_t t_min = minTimeNs();
        const int64_t t_max = maxTimeNs();
        if (t_ns < t_min || t_ns > t_max) {
            RESPLE_LOG_INVARIANT_ONCE("prepareInterpolation t_ns=" << t_ns << " outside ["
                << t_min << "," << t_max << "] (num_knot=" << num_knot
                << "); clamping to spline range");
            t_ns = std::max(t_min, std::min(t_max, t_ns));
        }
        int64_t t_ns_rel = t_ns - start_t_ns;
        int idx_l = floor(double(t_ns_rel) / double(dt_ns));
        idx_r = idx_l + 1;
        idx0 = std::max(idx_l - 2, 0);

        for (int i = 0; i < 2 - idx_l; i++) {
            cps[i] = knots_idle[i + idx_l + 1];
        }
        int idx_window = std::max(0, 2 - idx_l);
        // Defensive: cap n_active so idx0 + i cannot exceed num_knot, in case
        // the upstream clamp is bypassed (e.g. by a future caller that holds
        // a stale t_ns across a deque shrink). knots.size() == num_knot.
        const int n_active_safe =
            std::min<int>(std::min(idx_l + 2, 4),
                          std::max<int>(0, static_cast<int>(knots.size()) - static_cast<int>(idx0)));
        for (int i = 0; i < n_active_safe; i++) {
            cps[i + idx_window] = knots[idx0 + i];
        }
        u = (t_ns - start_t_ns - idx_l * dt_ns) / double(dt_ns);
    }

    template <int Derivative, class Derived>
    static void baseCoeffsWithTime(const Eigen::MatrixBase<Derived>& res_const, double t)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 4);
        Eigen::MatrixBase<Derived>& res = const_cast<Eigen::MatrixBase<Derived>&>(res_const);
        res.setZero();
        res[Derivative] = base_coefficients(Derivative, Derivative);
        double ti = t;
        for (int j = Derivative + 1; j < 4; j++) {
            res[j] = base_coefficients(Derivative, j) * ti;
            ti = ti * t;
        }
    }

    template <bool _Cumulative = false>
    static Eigen::Matrix4d computeBlendingMatrix()
    {
        Eigen::Matrix4d m;
        m.setZero();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double sum = 0;
                for (int s = j; s < 4; ++s) {
                    sum += std::pow(-1.0, s - j) * binomialCoefficient(4, s - j) *
                    std::pow(4 - s - 1.0, 4 - 1.0 - i);
                }
                m(j, i) = binomialCoefficient(3, 3 - i) * sum;
            }
        }
        if (_Cumulative) {
            for (int i = 0; i < 4; i++) {
                for (int j = i + 1; j < 4; j++) {
                    m.row(i) += m.row(j);
                }
            }
        }
        uint64_t factorial = 1;
        for (int i = 2; i < 4; ++i) {
            factorial *= i;
        }
        return m / factorial;
    }

    constexpr static inline uint64_t binomialCoefficient(uint64_t n, uint64_t k)
    {
        if (k > n) return 0;
        uint64_t r = 1;
        for (uint64_t d = 1; d <= k; ++d) {
            r *= n--;
            r /= d;
        }
        return r;
    }

    static Eigen::Matrix4d computeBaseCoefficients()
    {
        Eigen::Matrix4d base_coeff;
        base_coeff.setZero();
        base_coeff.row(0).setOnes();
        int order = 3;
        for (int n = 1; n < 4; n++) {
            for (int i = 3 - order; i < 4; i++) {
                base_coeff(n, i) = (order - 3 + i) * base_coeff(n - 1, i);
            }
            order--;
        }
        return base_coeff;
    }
};

inline const Eigen::Matrix4d SplineState::base_coefficients = SplineState::computeBaseCoefficients();
inline const Eigen::Matrix4d SplineState::blending_matrix = SplineState::computeBlendingMatrix();
inline const Eigen::Matrix4d SplineState::cumulative_blending_matrix = SplineState::computeBlendingMatrix<true>();