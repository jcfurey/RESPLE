#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <limits>
#include "SplineState.h"
#include "Association.h"
#include "utils/geometry_core.h"
#include "utils/overload_control.h"
#include "utils/lidar_gate.h"
#include "utils/range_weighting.h"

template<int XSIZE>
class Estimator
{
  public:
    static const int CP_SIZE = 24;
    static const int BA_OFFSET = 24;
    static const int BG_OFFSET = 27;
    int n_iter = 1;

    // HARDENING §3.2: correspondence thresholds + per-update funnel counters.
    // corresp_cfg is set once by the node at configure (like n_iter);
    // corresp_stats is reset at the top of each updateIEKF* call and
    // accumulated across its inner iterations. Both are touched on the worker
    // thread only (the node snapshots stats right after the update returns),
    // so neither is atomic by design.
    Association::CorrespConfig corresp_cfg;
    Association::CorrespStats corresp_stats;

    // HARDENING §3.2 robust kernel (M-estimator) on the point-to-plane
    // residuals: the IRLS weight w(zp) scales each point's information
    // (cov_inv_buf_), smoothly downweighting outliers instead of the binary
    // pt_thresh/cov_thresh accept-reject cliff alone (which is kept — the
    // kernel softens what survives it). 0 = none (legacy), 1 = Huber,
    // 2 = Cauchy; robust_delta is the kernel scale in meters (residual
    // units). Adaptive shape estimation (Barron, arXiv:2004.14938) is a
    // possible follow-up; fixed kernels first, per the decision-gate rule.
    // Set once at configure (like corresp_cfg); worker-thread reads only.
    int robust_kernel = 0;
    double robust_delta = 0.1;
    // §3.2 prototype: X-ICP-style degenerate-direction gate (translation
    // only). When the per-point-normalized LiDAR constraint matrix
    // E_tt = (1/N) Σ n nᵀ (world-frame normals) has an eigenvalue below this
    // gate, the LiDAR columns of the Kalman gain are projected out of that
    // eigendirection for the RCP position rows: the LiDAR update can no
    // longer pull the pose along an axis it cannot observe (tunnel map-lock),
    // while IMU rows and the spline prior keep full authority there. Joseph
    // form stays consistent for the projected (suboptimal) gain, so the
    // posterior covariance keeps growing along the gated axis — NIS and
    // downstream consumers see honest uncertainty. 0 disables (default;
    // decision-gate rule — see HARDENING §3.2/§6.3). E_tt eigenvalues sum
    // to 1; healthy geometry runs ~0.08+ min-eig, tunnel collapse is 1e-3
    // and below (06042026 analysis) — 0.02 is the recommended trial gate.
    double loc_gate_trans_min_eig = 0.0;
    // Publish-side covariance inflation along persistently-gated axes. The gate
    // above keeps the LiDAR from pulling along an unobservable axis, but the
    // IEKF posterior over the 4-knot window only reports cm-scale *local*
    // uncertainty there — it does not capture the metres-scale *global* drift
    // that accumulates while the axis stays unobserved (06042026 verify,
    // 2026-06-12). So a downstream EKF sees an overconfident /odom covariance.
    // These knobs grow an ADVISORY inflation that publishPoseAndTf adds to the
    // OUTGOING pose covariance only (the internal cov_rcp / gain are untouched —
    // trajectory is unchanged). A leaky integrator builds variance along the
    // gated subspace while the gate fires on consecutive frames; a persistence
    // guard keeps transient healthy arming (HelmDyn arms ~150k iters) from
    // inflating. 0 = off (default; decision-gate rule, pairs with the gate).
    double loc_gate_cov_rate = 0.0;      // m² added per severely-degenerate frame
    double loc_gate_cov_decay = 0.99;    // per-frame leak; steady state = rate/(1-decay)
    int    loc_gate_cov_persist = 5;     // consecutive degenerate frames before inflating
    // Inflation fires on a STRICTER eigenvalue threshold than the projection
    // gate: the gate must be lenient to catch degeneracy early (0.10 caught the
    // tunnel with 0 catastrophes) but it also arms on healthy geometry whose
    // min-eig dips to ~0.05-0.10, so reusing it would balloon healthy covariance.
    // True collapse is ~1e-3, so inflation triggers only below this threshold,
    // weighted by the eigenvalue deficit. 0 = derive it as loc_gate_trans_min_eig/10.
    double loc_gate_cov_min_eig = 0.0;
    // Diagnostics accessors for the gate (worker-thread reads, like
    // corresp_stats: the node snapshots right after the update returns).
    int locGateAxes() const { return loc_gate_axes_; }
    uint64_t locGateUpdateCount() const { return loc_gate_update_count_; }
    // World-frame translation inflation currently added to the published pose
    // covariance, and its largest standard deviation (m) for diagnostics.
    const Eigen::Matrix3d& locGateCovInfl() const { return loc_gate_cov_infl_; }
    double locGateCovInflMaxStd() const {
        if (loc_gate_cov_rate <= 0.0) return 0.0;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(loc_gate_cov_infl_,
                                                          Eigen::EigenvaluesOnly);
        return std::sqrt(std::max(0.0, es.eigenvalues()(2)));
    }
    double robustWeight(double r) const
    {
        const double a = std::abs(r);
        if (robust_kernel == 1) {  // Huber
            return (a <= robust_delta) ? 1.0 : robust_delta / a;
        }
        if (robust_kernel == 2) {  // Cauchy
            const double t = a / robust_delta;
            return 1.0 / (1.0 + t * t);
        }
        return 1.0;
    }

    // Close-range measurement down-weighting (commit 3bffada): variance is
    // inflated by (range_ref/r)^2 below range_ref so one nearby surface
    // cannot dominate H^T R^{-1} H in an otherwise-open scene. Parameterized
    // (2026-07-02 narrow-tunnel follow-up): the protection is ABSOLUTE, so in
    // a narrow tunnel where every return is close it degenerates into uniform
    // information starvation — walls at 1 m cut ALL LiDAR information 9x, at
    // 0.5 m 36x, and the filter coasts on the prior/IMU then snaps ("freaks
    // out"). Narrow-space configs should lower range_ref (or set 0 = off).
    // range_noise_scale_max bounds the inflation; the default 900 reproduces
    // the old implicit (3/0.1)^2 ceiling from the 0.1 m range floor. Set once
    // at configure (like corresp_cfg); worker-thread reads only.
    double range_ref = 3.0;
    double range_noise_scale_max = 900.0;
    // false = absolute (legacy, bit-identical). true = per-scan RELATIVE: the
    // inflation is normalized by the scan's farthest valid return, so a
    // confined space (tunnel / culvert / truck bed, where EVERY return is
    // close) is no longer uniformly starved of LiDAR information while the
    // anti-domination behaviour in open scenes is preserved exactly. See
    // utils/range_weighting.h for the derivation. Set once at configure.
    bool range_noise_relative = false;

    // Absolute inflation factor for one return (utils/range_weighting.h).
    double rangeNoiseScale(float range_sensor) const
    {
        return resple::range::rangeNoiseScale(rangeNoiseConfig(), range_sensor);
    }

    // Largest range among the returns fed to this update. NOTE: this filters on
    // `if_valid` (findCorresp accepted the correspondence) — it does NOT know
    // about the later per-row accept gate in updateLiDAR*, so a row that ends
    // up all-zero can still set the reference. Deliberate: if_valid is what is
    // known at this point, and a slightly larger reference only makes the
    // relative down-weighting more conservative.
    static float maxValidRange(const Eigen::aligned_deque<PointData>& pt_meas)
    {
        float r_max = 0.f;
        for (const auto& pt : pt_meas) {
            if (pt.if_valid && pt.range_sensor > r_max) r_max = pt.range_sensor;
        }
        return r_max;
    }

    // Overload hardening (2026-07-07): damping schedule for propRCP's
    // constant-velocity extrapolation across data gaps. Defaults (decay 1.0)
    // are a bit-identical no-op — the legacy expression is taken verbatim.
    // Set once at configure (params gap_extrap_decay / gap_extrap_free_knots,
    // pattern of corresp_cfg); worker-thread reads only.
    resple::overload::GapExtrapDamping gap_extrap;

    // Adaptive (Mahalanobis) LiDAR outlier gate. sigma == 0 (default) is a
    // bit-identical no-op: the legacy accept test decides every row on its own.
    // Set once at configure (params lidar_gate_sigma /
    // lidar_gate_max_reject_frac, pattern of corresp_cfg); worker reads only.
    // See utils/lidar_gate.h for why an ABSOLUTE residual threshold cannot do
    // this job, and how the disarm escape prevents a gating latch-up.
    resple::gate::MahalanobisGate lidar_gate;

    // Cumulative rows dropped by the gate; updates where it disarmed (the
    // rejected fraction exceeded the limit, so everything was admitted); and
    // rows admitted ONLY by the `lid_cov < var_pt·coeff_cov` disjunct while the
    // nn_thresh test said no. The last one is pure instrumentation and is
    // counted whether or not the gate is enabled — it measures how often the
    // shipped accept test short-circuits, which is the whole reason this gate
    // exists. Atomic: the diagnostics path reads them off the executor thread.
    uint64_t mahalRejected() const { return mahal_rejected_.load(std::memory_order_relaxed); }
    uint64_t mahalDisarms() const { return mahal_disarms_.load(std::memory_order_relaxed); }
    uint64_t covEscapeAdmits() const { return cov_escape_admits_.load(std::memory_order_relaxed); }

    // Cumulative knots extrapolated beyond the free window (i.e. genuine gap
    // fast-forwards, counted whether or not damping is active). Atomic because
    // the diagnostics path reads it from the node while the worker is mid-cycle.
    uint64_t gapExtrapKnots() const {
        return gap_extrap_knots_.load(std::memory_order_relaxed);
    }

    // Atomic because updateDiagnostics may read these from the ROS executor
    // thread while the worker is mid-IEKF. Monotonic; no reset — the diagnostics
    // publisher snapshots deltas.
    std::atomic<uint64_t> num_numerical_failures_{0};

    // NIS (normalized innovation squared) of the most recent update() call,
    // consumed by the HARDENING §3.3 divergence detector
    // (utils/filter_health.h). NaN when that update was skipped for numerical
    // reasons — the detector counts a non-finite NIS as a breach. Written and
    // read on the worker thread only (not atomic by design).
    double lastNis() const { return last_nis_; }
    int lastNisDof() const { return last_nis_dof_; }

    // IEKF iterations the last updateIEKF* call actually ran. The diagnostics
    // used to accumulate the `n_iter` PARAMETER (a constant 1 in the shipped
    // config) instead, so "avg IEKF iterations" under-reported by ~2x: with
    // n_iter=1 the loop's exit needs t>1, i.e. two associations and two
    // update() calls on the normal path. Worker-thread only, like corresp_stats.
    int lastIterations() const { return last_iterations_; }

    // True when the last updateIEKF* call actually folded a measurement batch
    // into the state (at least one successful update()). False when the frame
    // found zero correspondences or bailed on a numerical failure, i.e. when
    // the measurements it was handed were NOT consumed. The worker uses this to
    // decide whether the IMU staging deque may be cleared (consume-once): a
    // frame that never ran an update must keep its samples for the next pass.
    // Worker-thread only, like lastNis()/lastIterations().
    bool lastUpdatePerformed() const { return last_update_performed_; }

    // HARDENING SS3.3 'reset' recovery (bug A2): reinflate the IEKF covariance
    // to a genuine recovery covariance while keeping the state (spline +
    // biases). NIS divergence signals an OVER-confident (too-small) covariance,
    // so the recovery target must be LARGE — resetting to the tiny startup
    // prior (cov_P0 ~ 2e-6, sigma ~1.4 mm) would deflate it further, drive the
    // Kalman gain to ~0, and freeze the filter on the diverged state. cov_reset_
    // defaults to 1.0·I (sigma ~1 m / 1 rad), tunable via setRecoveryCovariance()
    // (node param nis_reset_cov). Worker-thread only (same thread as updateIEKF*).
    void reinflateCovariance() { cov_rcp = cov_reset_; }

    // Set the covariance the 'reset' recovery reinflates to (uniform variance on
    // the full state). Call once at configure; setState() does not touch it.
    void setRecoveryCovariance(double var) {
        cov_reset_ = var * Eigen::Matrix<double, XSIZE, XSIZE>::Identity();
    }

    Estimator() {};

    void setState(int64_t dt_ns, int64_t start_t_ns, const Eigen::Vector3d& t0, const Eigen::Quaterniond& q0, 
        const Eigen::Matrix<double, XSIZE, XSIZE>& Q, const Eigen::Matrix<double, XSIZE, XSIZE>& P)
    {
        spl.init(dt_ns, 0, start_t_ns, 0, t0, q0);
        for (int i = 0; i < 4; i++) {
            spl.addOneStateKnot(t0, Eigen::Vector3d::Zero());
        }        
        cov_sys = Q;
        cov_rcp = P;
        // Fresh filter: no edge has been charged process noise yet, so the first
        // zero-time propRCP after init injects (see propRCP).
        cov_sys_edge_ns_ = std::numeric_limits<int64_t>::min();
        a_mat = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
        Eigen::Matrix<double, 6, 6> matblock = Eigen::Matrix<double, 6, 6>::Zero();
        matblock.topLeftCorner<3, 3>().setIdentity();
        matblock.bottomRightCorner<3, 3>().setIdentity();

        a_mat.block(0, 6, 6, 6) = matblock;
        a_mat.block(6, 12, 6, 6) = matblock;
        a_mat.block(12, 18, 6, 6) = matblock;
        a_mat.block(18, 0, 3, 3) = - Eigen::Matrix3d::Identity();
        a_mat.block(18, 12, 3, 3) = 2 * Eigen::Matrix3d::Identity();
        a_mat.block(21, 9, 3, 3) = Eigen::Matrix3d::Identity();
        if constexpr (XSIZE == 30) {
            a_mat.block(BA_OFFSET, BA_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
            a_mat.block(BG_OFFSET, BG_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
        }
        // A (re)initialized filter must not inherit the previous run's
        // degenerate-direction state: a stale loc_gate_cov_infl_ would keep
        // inflating the published covariance of a fresh, healthy filter until
        // the leaky integrator drains it.
        loc_gate_cov_infl_.setZero();
        loc_gate_VVt_infl_.setZero();
        loc_gate_VVt_.setZero();
        loc_gate_persist_ctr_ = 0;
        loc_gate_infl_axes_ = 0;
        loc_gate_axes_ = 0;
        loc_gate_armed_ = false;
        loc_gate_mask_.resize(0);
    }

    Eigen::Matrix<double, XSIZE, 1> getState()
    {
        Eigen::Matrix<double, CP_SIZE, 1> cps_win = spl.getRCPs();
        Eigen::Matrix<double, XSIZE, 1> state;
        if constexpr (XSIZE == 24) {
            state = cps_win;
        } else {
            state << cps_win, ba, bg;
        }
        return state;
    }  

    void updateIEKFLiDAR(Eigen::aligned_deque<PointData>& pt_meas,
                         std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
                         KD_TREE<pcl::PointXYZINormal>* ikdtree, const double pt_thresh, const double cov_thresh,
                          int num_threads = 5, int num_match_points = 5)
    {
        corresp_stats.reset();
        // HARDENING §3.3 (bug A1): pessimistic per-frame NIS default. last_nis_
        // is written only inside update(); a frame that finds zero valid
        // correspondences breaks out below WITHOUT calling update(), so without
        // this reset lastNis() would return the PREVIOUS frame's (healthy) value
        // and the divergence detector would miss a lost-correspondence frame —
        // exactly the case it exists to catch.
        //
        // What the NaN actually does, precisely (the earlier claim here that it
        // "feeds the detector a breach via its non-finite path" was WRONG):
        // NisDivergenceDetector::feed returns early on dof <= 0, BEFORE its
        // non-finite check, and a frame that never calls update() reports
        // dof = 0 alongside the NaN. So such a frame is IGNORED by the
        // detector, not counted as a breach — which is the behaviour we want
        // (a legitimate correspondence dropout, e.g. occlusion, is an absence
        // of information, not evidence of over-confidence; counting it would be
        // a false positive). The NaN's real job is to stop lastNis() returning
        // the PREVIOUS frame's healthy value, which would let a stale reading
        // be attributed to this frame.
        //
        // The non-finite path is still reachable and still meaningful: an
        // update that RAN (dof > 0) but produced a non-finite NIS is a genuine
        // breach. Total dropout is deliberately NOT the detector's
        // jurisdiction; it is observable via corresp_used in the Diagnostics
        // message and gated in scripts/e2e_smoke_check.py.
        last_nis_ = std::numeric_limits<double>::quiet_NaN();
        last_nis_dof_ = 0;
        last_update_performed_ = false;
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        bool updated = false;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            last_iterations_ = i + 1;
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, ikdtree, pt_meas, pt_neighbors, corresp_cfg, &corresp_stats, num_threads, num_match_points);
            }
            if (num_tot_eff > 0) {
                if (!updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads)) {
                    std::cerr << "[Estimator] IEKF LiDAR update failed (numerical), resetting covariance to prior\n";
                    num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                    cov_rcp = cov_prop;
                    break;
                }
            } else {
                // Zero correspondences THIS iteration. If an earlier iteration
                // already applied an update, copy its Joseph-form posterior out
                // before breaking — leaving the propagated prior would pair an
                // updated state with a covariance that says nothing happened.
                if (updated) {
                    cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                }
                break;
            }
            updated = true;
            last_update_performed_ = true;
            converged = true;
            Eigen::Matrix<double, XSIZE, 1> state_af = getState();
            if ((state_af - rcpi).norm() > eps) {
                converged = false;
            } else {
                t++;
            }
            if(!t && i == max_iter - 2) {
                converged = true;
            }
            if ((t > n_iter) || (i == max_iter - 1)) {
                // Joseph-form posterior was computed inside update(); just copy it out.
                // Symmetrize defensively to clean up any FP roundoff from the matrix products.
                cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                break;
            }
        }
        // Advisory pose-cov inflation: only on frames that ran an update. A
        // zero-correspondence frame carries no observability information, so
        // the leaky integrator freezes rather than growing or decaying on the
        // PREVIOUS frame's stale degeneracy verdict. (Total dropout is the NIS
        // detector's jurisdiction, not the gate's.)
        if (updated) {
            accumGateInflation();
        }
    }

#ifdef RESPLE_USE_CUDA
    // GPU overload: same logic, but findCorresp uses the batched CudaMap path.
    void updateIEKFLiDAR(Eigen::aligned_deque<PointData>& pt_meas,
                         std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
                         resple_gpu::CudaMap* cuda_map, const double pt_thresh, const double cov_thresh,
                         int num_threads = 5, int num_match_points = 5)
    {
        corresp_stats.reset();
        // HARDENING §3.3 (bug A1): pessimistic per-frame NIS default. last_nis_
        // is written only inside update(); a frame that finds zero valid
        // correspondences breaks out below WITHOUT calling update(), so without
        // this reset lastNis() would return the PREVIOUS frame's (healthy) value
        // and the divergence detector would miss a lost-correspondence frame —
        // exactly the case it exists to catch.
        //
        // What the NaN actually does, precisely (the earlier claim here that it
        // "feeds the detector a breach via its non-finite path" was WRONG):
        // NisDivergenceDetector::feed returns early on dof <= 0, BEFORE its
        // non-finite check, and a frame that never calls update() reports
        // dof = 0 alongside the NaN. So such a frame is IGNORED by the
        // detector, not counted as a breach — which is the behaviour we want
        // (a legitimate correspondence dropout, e.g. occlusion, is an absence
        // of information, not evidence of over-confidence; counting it would be
        // a false positive). The NaN's real job is to stop lastNis() returning
        // the PREVIOUS frame's healthy value, which would let a stale reading
        // be attributed to this frame.
        //
        // The non-finite path is still reachable and still meaningful: an
        // update that RAN (dof > 0) but produced a non-finite NIS is a genuine
        // breach. Total dropout is deliberately NOT the detector's
        // jurisdiction; it is observable via corresp_used in the Diagnostics
        // message and gated in scripts/e2e_smoke_check.py.
        last_nis_ = std::numeric_limits<double>::quiet_NaN();
        last_nis_dof_ = 0;
        last_update_performed_ = false;
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        bool updated = false;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            last_iterations_ = i + 1;
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, cuda_map, pt_meas, pt_neighbors, corresp_cfg, &corresp_stats, num_threads, num_match_points);
            }
            if (num_tot_eff > 0) {
                if (!updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads)) {
                    std::cerr << "[Estimator] IEKF LiDAR update failed (numerical), resetting covariance to prior\n";
                    num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                    cov_rcp = cov_prop;
                    break;
                }
            } else {
                // Zero correspondences THIS iteration. If an earlier iteration
                // already applied an update, copy its Joseph-form posterior out
                // before breaking — leaving the propagated prior would pair an
                // updated state with a covariance that says nothing happened.
                if (updated) {
                    cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                }
                break;
            }
            updated = true;
            last_update_performed_ = true;
            converged = true;
            Eigen::Matrix<double, XSIZE, 1> state_af = getState();
            if ((state_af - rcpi).norm() > eps) {
                converged = false;
            } else {
                t++;
            }
            if(!t && i == max_iter - 2) {
                converged = true;
            }
            if ((t > n_iter) || (i == max_iter - 1)) {
                cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                break;
            }
        }
        // Advisory pose-cov inflation: only on frames that ran an update. A
        // zero-correspondence frame carries no observability information, so
        // the leaky integrator freezes rather than growing or decaying on the
        // PREVIOUS frame's stale degeneracy verdict. (Total dropout is the NIS
        // detector's jurisdiction, not the gate's.)
        if (updated) {
            accumGateInflation();
        }
    }

    void updateIEKFLiDARInertial(Eigen::aligned_deque<PointData>& pt_meas,
        std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
        resple_gpu::CudaMap* cuda_map, const double pt_thresh,
        Eigen::aligned_deque<ImuData>& imu_meas, const Eigen::Vector3d& g, const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyro, const double cov_thresh,
        int num_threads = 5, int num_match_points = 5)
    {
        corresp_stats.reset();
        // HARDENING §3.3 (bug A1): pessimistic per-frame NIS default. last_nis_
        // is written only inside update(); a frame that finds zero valid
        // correspondences breaks out below WITHOUT calling update(), so without
        // this reset lastNis() would return the PREVIOUS frame's (healthy) value
        // and the divergence detector would miss a lost-correspondence frame —
        // exactly the case it exists to catch.
        //
        // What the NaN actually does, precisely (the earlier claim here that it
        // "feeds the detector a breach via its non-finite path" was WRONG):
        // NisDivergenceDetector::feed returns early on dof <= 0, BEFORE its
        // non-finite check, and a frame that never calls update() reports
        // dof = 0 alongside the NaN. So such a frame is IGNORED by the
        // detector, not counted as a breach — which is the behaviour we want
        // (a legitimate correspondence dropout, e.g. occlusion, is an absence
        // of information, not evidence of over-confidence; counting it would be
        // a false positive). The NaN's real job is to stop lastNis() returning
        // the PREVIOUS frame's healthy value, which would let a stale reading
        // be attributed to this frame.
        //
        // The non-finite path is still reachable and still meaningful: an
        // update that RAN (dof > 0) but produced a non-finite NIS is a genuine
        // breach. Total dropout is deliberately NOT the detector's
        // jurisdiction; it is observable via corresp_used in the Diagnostics
        // message and gated in scripts/e2e_smoke_check.py.
        last_nis_ = std::numeric_limits<double>::quiet_NaN();
        last_nis_dof_ = 0;
        last_update_performed_ = false;
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        bool updated = false;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            last_iterations_ = i + 1;
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, cuda_map, pt_meas, pt_neighbors, corresp_cfg, &corresp_stats, num_threads, num_match_points);
            }
            bool update_ok = false;
            if (num_tot_eff > 0 && imu_meas.empty()) {
                update_ok = updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads);
            } else if (num_tot_eff > 0) {
                update_ok = updateLiDARInertial(pt_meas, imu_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, g, cov_acc, cov_gyro, num_threads);
            } else {
                // Zero correspondences THIS iteration — same posterior copy-out
                // rationale as the LO variant above.
                if (updated) {
                    cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                }
                break;
            }
            if (!update_ok) {
                std::cerr << "[Estimator] IEKF LIO update failed (numerical), resetting covariance to prior\n";
                num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                cov_rcp = cov_prop;
                break;
            }
            updated = true;
            last_update_performed_ = true;
            converged = true;
            Eigen::Matrix<double, XSIZE, 1> state_af = getState();
            if ((state_af - rcpi).norm() > eps) {
                converged = false;
            } else {
                t++;
            }
            if(!t && i == max_iter - 2) {
                converged = true;
            }
            if ((t > n_iter) || (i == max_iter - 1)) {
                cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                break;
            }
        }
        // Advisory pose-cov inflation: only on frames that ran an update. A
        // zero-correspondence frame carries no observability information, so
        // the leaky integrator freezes rather than growing or decaying on the
        // PREVIOUS frame's stale degeneracy verdict. (Total dropout is the NIS
        // detector's jurisdiction, not the gate's.)
        if (updated) {
            accumGateInflation();
        }
    }
#endif  // RESPLE_USE_CUDA

    void updateIEKFLiDARInertial(Eigen::aligned_deque<PointData>& pt_meas,
        std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
        KD_TREE<pcl::PointXYZINormal>* ikdtree, const double pt_thresh,
        Eigen::aligned_deque<ImuData>& imu_meas, const Eigen::Vector3d& g, const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyro, const double cov_thresh,
        int num_threads = 5, int num_match_points = 5)
    {
        corresp_stats.reset();
        // HARDENING §3.3 (bug A1): pessimistic per-frame NIS default. last_nis_
        // is written only inside update(); a frame that finds zero valid
        // correspondences breaks out below WITHOUT calling update(), so without
        // this reset lastNis() would return the PREVIOUS frame's (healthy) value
        // and the divergence detector would miss a lost-correspondence frame —
        // exactly the case it exists to catch.
        //
        // What the NaN actually does, precisely (the earlier claim here that it
        // "feeds the detector a breach via its non-finite path" was WRONG):
        // NisDivergenceDetector::feed returns early on dof <= 0, BEFORE its
        // non-finite check, and a frame that never calls update() reports
        // dof = 0 alongside the NaN. So such a frame is IGNORED by the
        // detector, not counted as a breach — which is the behaviour we want
        // (a legitimate correspondence dropout, e.g. occlusion, is an absence
        // of information, not evidence of over-confidence; counting it would be
        // a false positive). The NaN's real job is to stop lastNis() returning
        // the PREVIOUS frame's healthy value, which would let a stale reading
        // be attributed to this frame.
        //
        // The non-finite path is still reachable and still meaningful: an
        // update that RAN (dof > 0) but produced a non-finite NIS is a genuine
        // breach. Total dropout is deliberately NOT the detector's
        // jurisdiction; it is observable via corresp_used in the Diagnostics
        // message and gated in scripts/e2e_smoke_check.py.
        last_nis_ = std::numeric_limits<double>::quiet_NaN();
        last_nis_dof_ = 0;
        last_update_performed_ = false;
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        bool updated = false;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            last_iterations_ = i + 1;
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, ikdtree, pt_meas, pt_neighbors, corresp_cfg, &corresp_stats, num_threads, num_match_points);
            }
            bool update_ok = false;
            if (num_tot_eff > 0 && imu_meas.empty()) {
                update_ok = updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads);
            } else if (num_tot_eff > 0) {
                update_ok = updateLiDARInertial(pt_meas, imu_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, g, cov_acc, cov_gyro, num_threads);
            } else {
                // Zero correspondences THIS iteration — same posterior copy-out
                // rationale as the LO variant above.
                if (updated) {
                    cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                }
                break;
            }
            if (!update_ok) {
                std::cerr << "[Estimator] IEKF LIO update failed (numerical), resetting covariance to prior\n";
                num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                cov_rcp = cov_prop;
                break;
            }
            updated = true;
            last_update_performed_ = true;
            converged = true;
            Eigen::Matrix<double, XSIZE, 1> state_af = getState();
            if ((state_af - rcpi).norm() > eps) {
                converged = false;
            } else {
                t++;
            }
            if(!t && i == max_iter - 2) {
                converged = true;
            }
            if ((t > n_iter) || (i == max_iter - 1)) {
                // Joseph-form posterior was computed inside update(); just copy it out.
                // Symmetrize defensively to clean up any FP roundoff from the matrix products.
                cov_rcp = 0.5 * (cov_post_ + cov_post_.transpose());
                break;
            }
        }
        // Advisory pose-cov inflation: only on frames that ran an update. A
        // zero-correspondence frame carries no observability information, so
        // the leaky integrator freezes rather than growing or decaying on the
        // PREVIOUS frame's stale degeneracy verdict. (Total dropout is the NIS
        // detector's jurisdiction, not the gate's.)
        if (updated) {
            accumGateInflation();
        }
    }

    void propRCP(int64_t t)
    {
        if (spl.maxTimeNs() >= t) {
            // No knot added: the estimated state window is UNCHANGED, so this is
            // a zero-time propagation. Upstream injected cov_sys on every such
            // call, which made the prior's looseness a function of how many
            // measurement batches happened to land inside one knot interval —
            // and that count is set by num_points_upd versus the scan's point
            // density (so, indirectly, by scene geometry and CPU load). Two
            // batches per interval doubled the injected process noise, i.e. the
            // same bag ran with a different prior/measurement balance at a
            // different point density: a silent, load-dependent retune.
            //
            // Inject at most once per distinct spline edge. In the steady
            // one-batch-per-knot case this is bit-identical to the old code (the
            // growth propRCP in collectMeasurements advances the edge first, so
            // the following batch call always sees a new edge and injects);
            // additional batches inside the SAME interval now fold in
            // sequentially with no artificial inflation between them, which is
            // the correct treatment of independent measurements updating one
            // state window. maxTimeNs() is monotonic (pruning moves start_t_ns,
            // never the edge), so this cannot be fooled by an ABA edge.
            const int64_t edge = spl.maxTimeNs();
            if (edge != cov_sys_edge_ns_) {
                cov_rcp += cov_sys;
                cov_sys_edge_ns_ = edge;
            }
        } else {
            // k counts knots added by THIS call: steady state adds <= ~2 per
            // call, while a shed/dropped-scan gap is fast-forwarded in one
            // call — so gap_extrap.free_knots (default 3) exempts normal
            // operation and the damping schedule engages only on real gaps.
            int k = 0;
            while (spl.maxTimeNs() < t) {
                Eigen::Matrix<double, 24, 1> cps_win = spl.getRCPs();
                const double s = gap_extrap.scale(k);
                if (k >= gap_extrap.free_knots) {
                    gap_extrap_knots_.fetch_add(1, std::memory_order_relaxed);
                }
                Eigen::Vector3d cp_prop_pos;
                Eigen::Vector3d delta;
                if (s == 1.0) {
                    // Literal legacy expression (damping off / free window):
                    // stride-2 constant-velocity extrapolation, bit-identical
                    // to the pre-damping code path.
                    Eigen::Matrix<double, 6, 1> cp_prop = 2*cps_win.block<6, 1>(12, 0) - cps_win.block<6, 1>(0, 0);
                    cp_prop_pos = cp_prop.head<3>();
                    delta = cps_win.segment<3>(9);
                } else {
                    // Damped gap extrapolation (overload hardening): newest-
                    // knot form p3 + s·(p3 − p2). Identical to the legacy
                    // 2·p2 − p0 at constant velocity, but converges to a
                    // clean hold-at-newest as s→0 — damping the legacy form
                    // instead converges to a 2-cycle oscillation between the
                    // last two knot positions (pinned in
                    // test_overload_control.cpp). The rotation delta is
                    // damped toward zero the same way. Covariance propagation
                    // below is deliberately UNCHANGED (A·P·Aᵀ + Q per knot):
                    // uncertainty keeps growing across the gap even though
                    // the mean motion is damped.
                    const Eigen::Vector3d p3 = cps_win.block<3, 1>(18, 0);
                    const Eigen::Vector3d p2 = cps_win.block<3, 1>(12, 0);
                    cp_prop_pos = p3 + s * (p3 - p2);
                    delta = s * cps_win.segment<3>(9);
                }
                spl.addOneStateKnot(cp_prop_pos, delta);
                cov_rcp = a_mat * cov_rcp * a_mat.transpose() + cov_sys;
                ++k;
            }
        }
    }

    SplineState* getSpline() {
        return &spl;
    }

    // Returns the full 6×6 IEKF posterior covariance for the interpolated pose at
    // maxTimeNs(), computed by propagating cov_rcp through the spline Jacobian:
    //   P_pose = J · cov_rcp · Jᵀ   (J is 6×24)
    //
    // Layout: [x, y, z, rx, ry, rz] — orientation in the body-frame right-perturbation
    // convention (δφ = 2·imag(q⁻¹ ⊗ δq)), matching ROS PoseWithCovarianceStamped.
    //
    // This correctly accounts for all four active knots' contributions to the
    // interpolated orientation (via the cumulative B-spline Jacobian), unlike a
    // direct block-extract which captures only the last knot's incremental uncertainty.
    Eigen::Matrix<double, 6, 6> getLastPoseCovariance() const {
        const int64_t t = spl.maxTimeNs();
        const int RCP_st_id = static_cast<int>(spl.numKnots()) - 4;

        // ── Position Jacobian (scalar blend coefficients) ──────────────────
        // At u≈1, blending weights are [0,0,0,1]: only RCP[3] contributes.
        Jacobian J_pos;
        spl.itpPosition(t, &J_pos);

        // ── Orientation Jacobian (4×3 per knot) ───────────────────────────
        // At u≈1, cumulative blending gives coeff=[1,1,1,1]: all four knots
        // contribute to the interpolated quaternion.
        Jacobian43 J_q;
        Eigen::Quaterniond q_out;
        spl.itpQuaternion(t, &q_out, nullptr, &J_q);

        // G (3×4): maps 4D δq [w,x,y,z] → a 3-vector of small rotations about the
        // FIXED (header.frame_id == odom) axes: 2·imag(δq ⊗ q_out⁻¹)
        // = 2·[rows 1,2,3 of Qright(q_out⁻¹)] · δq.
        //
        // This used to use the Qleft form, i.e. the BODY (right/local)
        // perturbation, while claiming to match PoseWithCovarianceStamped. That
        // message specifies a FIXED-AXIS representation, and robot_localization
        // rotates an incoming pose covariance only from the message frame into
        // its world frame — identity here, since frame_id is already odom — so
        // the body-frame block was consumed verbatim as world roll/pitch/yaw.
        // Anisotropy then landed on the wrong axes (over-trusting the least
        // observable one) and the position↔rotation cross blocks mixed frames.
        // Using the world map fixes the rotation block AND the cross blocks in
        // one step, since P_world = R·P_body·Rᵗ (see utils/geometry_core.h).
        // The position block was, and remains, world-frame.
        const Eigen::Matrix<double, 3, 4> G = resple::geom::quatPerturbToWorld(q_out);

        // ── Assemble 6×24 Jacobian ─────────────────────────────────────────
        Eigen::Matrix<double, 6, 24> J = Eigen::Matrix<double, 6, 24>::Zero();
        for (int i = 0; i < static_cast<int>(J_pos.d_val_d_knot.size()); ++i) {
            const int j = static_cast<int>(J_pos.start_idx) + i - RCP_st_id;
            if (j >= 0 && j < 4)
                J.block<3, 3>(0, j * 6) =
                    J_pos.d_val_d_knot[i] * Eigen::Matrix3d::Identity();
        }
        for (int i = 0; i < static_cast<int>(J_q.d_val_d_knot.size()); ++i) {
            const int j = static_cast<int>(J_q.start_idx) + i - RCP_st_id;
            if (j >= 0 && j < 4)
                J.block<3, 3>(3, j * 6 + 3).noalias() = G * J_q.d_val_d_knot[i];
        }

        return J * cov_rcp.template topLeftCorner<24, 24>() * J.transpose();
    }

    // Returns the 6×6 IEKF posterior covariance for the body-frame twist at
    // maxTimeNs(), layout [vx, vy, vz, wx, wy, wz].
    //
    // Linear block:  v_body = R(q)ᵀ · v_world. We propagate cov_rcp through
    // the Jacobian of world-frame linear velocity (∂v_w/∂RCP via itpPosition<1>),
    // then rotate the resulting 3×3 into the body frame:
    //     P_v_body ≈ Rᵀ · (J_v · cov_rcp · J_vᵀ) · R
    // This is exact up to the secondary coupling between pose uncertainty and
    // the body-frame linear velocity (typically negligible for EKF fusion).
    //
    // Angular block: ω_body comes from the spline directly; J_ω is produced by
    // itpQuaternion as Jacobian33 entries (3×3 per knot):
    //     P_ω_body = J_ω · cov_rcp · J_ωᵀ
    //
    // Cross-terms (v,ω) are set to zero — robot_localization ignores them and
    // including a noisy approximation hurts fusion quality more than it helps.
    Eigen::Matrix<double, 6, 6> getLastTwistCovariance() const {
        const int64_t t = spl.maxTimeNs();
        const int RCP_st_id = static_cast<int>(spl.numKnots()) - 4;

        // ── Linear velocity Jacobian (world frame, 3×24) ───────────────────
        Jacobian J_vpos;
        spl.itpPosition<1>(t, &J_vpos);

        // ── Angular velocity Jacobian (body frame, 3×24) ──────────────────
        Jacobian33 J_w;
        Eigen::Quaterniond q_out;
        Eigen::Vector3d w_out;
        spl.itpQuaternion(t, &q_out, &w_out, nullptr, &J_w);

        Eigen::Matrix<double, 3, 24> Jv = Eigen::Matrix<double, 3, 24>::Zero();
        for (int i = 0; i < static_cast<int>(J_vpos.d_val_d_knot.size()); ++i) {
            const int j = static_cast<int>(J_vpos.start_idx) + i - RCP_st_id;
            if (j >= 0 && j < 4)
                Jv.block<3, 3>(0, j * 6) =
                    J_vpos.d_val_d_knot[i] * Eigen::Matrix3d::Identity();
        }

        Eigen::Matrix<double, 3, 24> Jw = Eigen::Matrix<double, 3, 24>::Zero();
        for (int i = 0; i < static_cast<int>(J_w.d_val_d_knot.size()); ++i) {
            const int j = static_cast<int>(J_w.start_idx) + i - RCP_st_id;
            if (j >= 0 && j < 4)
                Jw.block<3, 3>(0, j * 6 + 3) = J_w.d_val_d_knot[i];
        }

        const auto& P = cov_rcp.template topLeftCorner<24, 24>();
        const Eigen::Matrix3d P_vw = Jv * P * Jv.transpose();
        const Eigen::Matrix3d R = q_out.toRotationMatrix();
        const Eigen::Matrix3d P_v_body = R.transpose() * P_vw * R;
        const Eigen::Matrix3d P_w_body = Jw * P * Jw.transpose();

        Eigen::Matrix<double, 6, 6> P_twist = Eigen::Matrix<double, 6, 6>::Zero();
        P_twist.topLeftCorner<3, 3>()     = P_v_body;
        P_twist.bottomRightCorner<3, 3>() = P_w_body;
        return P_twist;
    }

  private:
    resple::range::RangeNoiseConfig rangeNoiseConfig() const
    {
        resple::range::RangeNoiseConfig cfg;
        cfg.range_ref = range_ref;
        cfg.scale_max = range_noise_scale_max;
        cfg.relative = range_noise_relative;
        return cfg;
    }

    SplineState spl;
    // Zero-initialized explicitly: the default workspace build uses
    // RelWithDebInfo which disables Eigen's optional NaN-init (Debug-only
    // EIGEN_INITIALIZE_MATRICES_BY_NAN). Previously these matrices held
    // whatever was on the heap until setState() / update() wrote them; any
    // read before that first write would have been UB. setState() now runs
    // on valid zeros.
    Eigen::Matrix<double, XSIZE, XSIZE> cov_rcp  = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    // Recovery covariance the HARDENING SS3.3 'reset' mode reinflates to
    // (reinflateCovariance / bug A2). Defaults to 1.0·I (sigma ~1 m / 1 rad);
    // override via setRecoveryCovariance() (node param nis_reset_cov). setState()
    // deliberately does NOT touch it, so a configure-time setter survives init.
    Eigen::Matrix<double, XSIZE, XSIZE> cov_reset_ = Eigen::Matrix<double, XSIZE, XSIZE>::Identity();
    Eigen::Matrix<double, XSIZE, XSIZE> cov_sys  = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    Eigen::Matrix<double, XSIZE, XSIZE> a_mat    = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, XSIZE, XSIZE> KH       = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    // Joseph-form posterior covariance set by update() each iteration.
    // P_post = (I - KH) P_prior (I - KH)^T + K R K^T -- numerically PSD-preserving
    // in floating point, unlike the simpler (I - KH) P_prior form.
    Eigen::Matrix<double, XSIZE, XSIZE> cov_post_ = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    // See lastNis(): measurement-space NIS nu^T S^{-1} nu of the latest
    // update(); NaN if that update failed. Written by update() only.
    double last_nis_ = std::numeric_limits<double>::quiet_NaN();
    int last_nis_dof_ = 0;
    // See lastIterations(): actual iteration count of the latest update.
    int last_iterations_ = 0;

    // See lastUpdatePerformed(): did the last updateIEKF* call consume its
    // measurement batch?
    bool last_update_performed_ = false;

    // Spline edge (maxTimeNs) most recently charged one cov_sys increment by a
    // zero-time propRCP. See propRCP for why this is per-edge and not per-call.
    int64_t cov_sys_edge_ns_ = std::numeric_limits<int64_t>::min();

    // Row indices the adaptive gate flagged in the current assembly pass. A
    // member (not a local) so the allocation is reused across updates —
    // clear() keeps the capacity. Worker thread only.
    std::vector<int> mahal_reject_;
    std::atomic<uint64_t> mahal_rejected_{0};
    std::atomic<uint64_t> mahal_disarms_{0};
    std::atomic<uint64_t> cov_escape_admits_{0};
    // See gapExtrapKnots(): knots propRCP added beyond the free window.
    std::atomic<uint64_t> gap_extrap_knots_{0};
    Eigen::Matrix<double, Eigen::Dynamic, XSIZE> H_buf_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> innv_buf_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> cov_inv_buf_;
    // Degenerate-direction gate state (worker-thread only, armed by the
    // updateLiDAR* assemblers immediately before update() consumes it).
    // loc_gate_mask_ marks which measurement rows are LiDAR (1) vs IMU (0):
    // the LIO assembler interleaves them by timestamp, so the projection
    // must select gain columns, not a contiguous block.
    Eigen::ArrayXd loc_gate_mask_;
    Eigen::Matrix3d loc_gate_VVt_ = Eigen::Matrix3d::Zero();
    bool loc_gate_armed_ = false;
    int loc_gate_axes_ = 0;            // degenerate axes at last arm check
    uint64_t loc_gate_update_count_ = 0;  // updates with projection applied
    // Publish-side inflation accumulator (worker-thread only; written by
    // accumGateInflation() once per frame, read by the node in publishPoseAndTf
    // under spline_mutex_).
    Eigen::Matrix3d loc_gate_cov_infl_ = Eigen::Matrix3d::Zero();
    // Deficit-weighted projector onto SEVERELY-degenerate directions (λ below
    // loc_gate_cov_min_eig), built in armLocGate alongside the projection VVt.
    Eigen::Matrix3d loc_gate_VVt_infl_ = Eigen::Matrix3d::Zero();
    int loc_gate_infl_axes_ = 0;          // severely-degenerate axes this frame
    int loc_gate_persist_ctr_ = 0;        // consecutive severely-degenerate frames

    // Once-per-frame leaky integrator for the advisory pose-covariance
    // inflation. loc_gate_infl_axes_ / loc_gate_VVt_infl_ hold the converged
    // iteration's SEVERE-degeneracy state (stricter than the projection gate) so
    // a tunnel inflates while healthy geometry — which trips the lenient gate but
    // not the strict inflation threshold — does not. A tunnel keeps the same axis
    // severely degenerate every frame → the counter passes the threshold and the
    // integrator builds toward rate/(1-decay) along loc_gate_VVt_infl_.
    void accumGateInflation()
    {
        if (loc_gate_cov_rate <= 0.0) return;            // feature off
        if (loc_gate_infl_axes_ > 0) {
            loc_gate_cov_infl_ *= loc_gate_cov_decay;
            if (++loc_gate_persist_ctr_ >= loc_gate_cov_persist)
                loc_gate_cov_infl_.noalias() += loc_gate_cov_rate * loc_gate_VVt_infl_;
        } else {
            loc_gate_persist_ctr_ = 0;
            loc_gate_cov_infl_ *= loc_gate_cov_decay;    // relax when observable
        }
    }

    // Finish the adaptive LiDAR outlier gate for one assembled measurement
    // block (utils/lidar_gate.h). The assemblers fill every row that passes the
    // legacy accept test and record which of them the gate flagged; this decides
    // whether the gate holds and, if so, zeroes the flagged rows.
    //
    // Zeroing IS rejection in this assembler: update() sums HᵗR⁻¹H and HᵗR⁻¹ν,
    // so an all-zero H row with a zero innovation contributes nothing to either,
    // and update()'s dof counter already excludes such rows from the NIS
    // statistic (see its comment). The row's cov_inv_buf_ entry is left as-is
    // for the same reason it is for legacy-rejected rows: it multiplies zeros.
    //
    // When the gate disarms (too large a rejected fraction — evidence against
    // the prior rather than the data) the flagged rows' loc-gate contributions
    // are folded back in, so the degeneracy verdict sees exactly the row set the
    // filter used. Nothing here runs when the gate is disabled.
    void applyMahalanobisGate(int accepted_rows,
                              const Eigen::Matrix3d& mahal_Ett, double mahal_w, int mahal_n,
                              Eigen::Matrix3d& gate_Ett, double& gate_w, int& gate_n)
    {
        if (mahal_reject_.empty()) return;
        if (!lidar_gate.staysArmed(static_cast<int>(mahal_reject_.size()), accepted_rows)) {
            gate_Ett.noalias() += mahal_Ett;
            gate_w += mahal_w;
            gate_n += mahal_n;
            mahal_disarms_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        for (int row : mahal_reject_) {
            H_buf_.row(row).setZero();
            innv_buf_(row) = 0.0;
            if (loc_gate_trans_min_eig > 0.0 && row < loc_gate_mask_.size()) {
                loc_gate_mask_(row) = 0.0;
            }
        }
        mahal_rejected_.fetch_add(mahal_reject_.size(), std::memory_order_relaxed);
    }

    // Decide whether this update's LiDAR constraint set is degenerate and
    // cache the projector. Ett_sum = Σ w·n nᵀ over the rows actually filled,
    // with w the relative information weight (1/rangeNoiseScale) so the gate
    // sees the same starvation the filter does; w_sum normalizes so the
    // eigenvalues still sum to 1 (unit normals), keeping the calibrated
    // thresholds valid. All-far scenes (w == 1) reduce to the old Σ/n form.
    void armLocGate(const Eigen::Matrix3d& Ett_sum, double w_sum, int n_used)
    {
        loc_gate_armed_ = false;
        loc_gate_axes_ = 0;
        loc_gate_infl_axes_ = 0;
        if (loc_gate_trans_min_eig <= 0.0 || n_used < 20 || w_sum <= 0.0) return;
        // Stricter, separate threshold for the advisory covariance inflation
        // (default: an order below the projection gate). Keeps healthy geometry
        // that merely trips the lenient gate from inflating the published cov.
        const double infl_thresh = loc_gate_cov_min_eig > 0.0
            ? loc_gate_cov_min_eig : 0.1 * loc_gate_trans_min_eig;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Ett_sum / w_sum);
        Eigen::Matrix3d VVt = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d VVt_infl = Eigen::Matrix3d::Zero();
        for (int i = 0; i < 3; i++) {
            const double lambda = es.eigenvalues()(i);
            if (lambda < loc_gate_trans_min_eig) {
                const Eigen::Vector3d v = es.eigenvectors().col(i);
                VVt.noalias() += v * v.transpose();
                loc_gate_axes_++;
                if (lambda < infl_thresh) {
                    // Deficit weight in [0,1): ~0 just under the threshold, →1 as
                    // λ→0, so only genuine collapse contributes meaningfully.
                    const double w = (infl_thresh - lambda) / infl_thresh;
                    VVt_infl.noalias() += w * v * v.transpose();
                    loc_gate_infl_axes_++;
                }
            }
        }
        if (loc_gate_axes_ > 0) {
            loc_gate_VVt_ = VVt;
            loc_gate_armed_ = true;
        }
        if (loc_gate_infl_axes_ > 0)
            loc_gate_VVt_infl_ = VVt_infl;
    }

    // Project the degenerate-direction component of the LiDAR columns out of
    // the RCP position rows of K. IMU columns (mask 0) keep full gain; bias
    // rows (XSIZE==30) are untouched.
    template <int RSIZE>
    void applyLocGate(Eigen::Matrix<double, XSIZE, RSIZE>& K)
    {
        for (int j = 0; j < 4; j++) {
            auto rows = K.template middleRows<3>(j * 6);
            const Eigen::Matrix<double, 3, Eigen::Dynamic> lidar_part =
                (rows.array().rowwise() * loc_gate_mask_.transpose()).matrix();
            rows.noalias() -= loc_gate_VVt_ * lidar_part;
        }
        loc_gate_update_count_++;
    }
    int max_iter = 5;
    double eps = 0.1;

    void prepIMU(ImuData& imu_data, const Eigen::Vector3d& g)
    {
        Eigen::Quaterniond q_itp;
        Eigen::Vector3d rot_vel;
        // thread_local for the same reason as prepLiDAR (see the note there):
        // these carry resize()d std::vectors and prepIMU also runs inside an
        // OpenMP parallel-for, one call per staged IMU sample.
        thread_local Jacobian43 J_ortdel;
        thread_local Jacobian J_line_acc;
        thread_local Jacobian33 J_gyro;
        spl.itpQuaternion(imu_data.time_ns, &q_itp, &rot_vel, &J_ortdel, &J_gyro);
        Eigen::Vector3d a_w_no_g = spl.itpPosition<2>(imu_data.time_ns, &J_line_acc);
        Eigen::Vector3d a_w = a_w_no_g + g;
        Eigen::Matrix3d RT = q_itp.inverse().toRotationMatrix();   
        Eigen::Matrix<double, 3, 4> drot;
        Quater::drot(a_w, q_itp, drot);
        Eigen::Matrix<double, 6, XSIZE> Hi = Eigen::Matrix<double, 6, XSIZE>::Zero();
        int recur_st_id = spl.numKnots() - 4;
        for (int i = 0; i < (int) J_line_acc.d_val_d_knot.size(); i++) {
            int j = J_line_acc.start_idx + i - recur_st_id;
            // Upper bound j < 4: Hi has only 4 knot-blocks. j is provably <= 3
            // today (itpPose clamps t_ns), but a regression in that clamp would
            // otherwise write j==4 onto the ba/bg bias columns (XSIZE==30) —
            // silent estimation corruption — or off the end (XSIZE==24).
            // Matches the explicit guard in getLastPose/TwistCovariance.
            if (j >= 0 && j < 4) {
                Hi.block(0, j*6, 3, 3) = RT * J_line_acc.d_val_d_knot[i];
                Hi.block(0, j*6 + 3, 3, 3) = drot * J_ortdel.d_val_d_knot[i];
                Hi.block(3, j*6 + 3, 3, 3) = J_gyro.d_val_d_knot[i];                
            }
        }       
        // Bias columns exist only in the LIO state (XSIZE==30). Guarded because
        // BA_OFFSET/BG_OFFSET are 24/27: on an Estimator<24> these blocks run
        // off the end of the 6xXSIZE matrix. Unreachable today (only the LIO
        // estimator takes the inertial path) but the guard costs nothing and
        // the failure mode under -DNDEBUG is a silent out-of-range write.
        if constexpr (XSIZE == 30) {
            Hi.block(0, BA_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
            Hi.block(3, BG_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
        }
        imu_data.imu_itp.head<3>() = RT * a_w + ba;
        imu_data.imu_itp.tail<3>() = rot_vel + bg;
        imu_data.H = Hi.template leftCols<24>();
    }    

    void prepLiDAR(PointData& pt_data) const
    {
        if (pt_data.if_valid) {
            Eigen::Matrix<double, 1, XSIZE> Hi = Eigen::Matrix<double, 1, XSIZE>::Zero();
            Eigen::Quaterniond q_itp;
            // thread_local, not plain locals: these hold std::vectors that
            // itpPose resize()s, so a fresh pair per point meant two
            // operator new + two frees for EVERY valid LiDAR point (measured
            // 791 allocations on a 300-point, 2-iteration update). Reusing
            // them makes resize() free after the first point in each thread.
            //
            // Safe against stale contents: both itpPose write sites fill ALL
            // size_J entries after the resize (J_p's forward loop and J_q's
            // reverse loop), and the defensive early-return paths clear() both
            // in lockstep — so nothing from the previous point survives into
            // this one. thread_local gives each OpenMP worker its own pair, so
            // the parallel-for over points stays race-free.
            thread_local Jacobian43 J_ortdel;
            thread_local Jacobian J_pos;
            Eigen::Vector3d p_itp;
            spl.itpPose(pt_data.time_ns, &p_itp, &J_pos, &q_itp, &J_ortdel);
            Eigen::Matrix3d R_IL = pt_data.q_bl.toRotationMatrix();
            Eigen::Vector3d pt_w = q_itp * (R_IL * pt_data.pt_b + pt_data.t_bl) + p_itp;
            pt_data.zp = pt_data.normvec.dot(pt_w) + pt_data.dist;

            Eigen::Matrix<double, 3, 4> drot;
            Quater::drotInv((R_IL * pt_data.pt_b + pt_data.t_bl), q_itp, drot);
            Eigen::Matrix<double, 1, 4> tmp = pt_data.normvec.transpose() * drot;
            int RCP_st_id = spl.numKnots() - 4;
            for (int i = 0; i < (int) J_pos.d_val_d_knot.size(); i++) {
                int j = (int) J_pos.start_idx + i - RCP_st_id;
                // Upper bound j < 4 (see prepIMU): defends the bias columns /
                // deque end against a clamp regression. Matches the diagnostic
                // covariance paths' guard.
                if (j >= 0 && j < 4) {
                    Hi.block(0, j*6, 1, 3) = pt_data.normvec.transpose() * J_pos.d_val_d_knot[i];
                    Hi.block(0, j*6 + 3, 1, 3) = tmp * J_ortdel.d_val_d_knot[i];
                }
            }  
            pt_data.H = Hi.template leftCols<24>();
        }
    } 

    void updateState(const Eigen::Matrix<double, XSIZE, 1>& xupd)
    {
        Eigen::Matrix<double, CP_SIZE, 1> cp_win = xupd.segment(0, CP_SIZE);
        spl.updateRCPs(cp_win);
        if constexpr (XSIZE == 30) {
            ba = xupd.segment(BA_OFFSET, 3);
            bg = xupd.segment(BG_OFFSET, 3);   
        }
    }        

    bool updateLiDAR(Eigen::aligned_deque<PointData>& pt_meas, int num_valid, const Eigen::Matrix<double, XSIZE, 1>& x_prop, 
        const Eigen::Matrix<double, XSIZE, XSIZE>& P_prop, const double pt_thresh, const double cov_thresh, int num_threads = 5)
    {
        // resize, not conservativeResize: all three are overwritten wholesale
        // on the next three lines, so preserving the old contents is pure
        // copying of data that is immediately discarded. Eigen's resize() is a
        // no-op when the size is unchanged (the steady-state case), so this
        // also avoids the reallocation conservativeResize can force.
        H_buf_.resize(num_valid, XSIZE);
        innv_buf_.resize(num_valid, 1);
        cov_inv_buf_.resize(num_valid, 1);
        H_buf_.setZero();
        innv_buf_.setZero();
        cov_inv_buf_.setConstant(1/0.01);
        size_t num_pt = pt_meas.size();
        #pragma omp parallel for num_threads(num_threads) schedule(dynamic)
        for (size_t i = 0; i < num_pt; i++) {
            PointData& pt_data = pt_meas[i];
            prepLiDAR(pt_data);
        }
        int idx_offset = 0;
        // Close-range down-weighting reference for THIS update. In relative
        // mode this anchors the inflation to the scan's farthest valid return
        // so a uniformly-close scene keeps full LiDAR authority; in absolute
        // (default) mode it is 1.0 and the arithmetic is unchanged.
        const resple::range::RangeNoiseConfig rn_cfg = rangeNoiseConfig();
        const double rn_norm =
            resple::range::rangeNoiseNormalizer(rn_cfg, maxValidRange(pt_meas));
        Eigen::Matrix3d gate_Ett = Eigen::Matrix3d::Zero();
        double gate_w = 0.0;
        int gate_n = 0;
        if (loc_gate_trans_min_eig > 0.0) {
            loc_gate_mask_ = Eigen::ArrayXd::Ones(num_valid);  // LO: all rows LiDAR
        }
        // Adaptive outlier gate (utils/lidar_gate.h). Rows are assembled
        // optimistically and the flagged ones are zeroed after the pass, IF the
        // gate holds — so the escape hatch costs nothing and the disabled case
        // (default) never touches any of this.
        mahal_reject_.clear();
        Eigen::Matrix3d mahal_Ett = Eigen::Matrix3d::Zero();
        double mahal_w = 0.0;
        int mahal_n = 0;
        int accepted_rows = 0;
        for(size_t i = 0; i < num_pt; i++) {
            const PointData& pt_data = pt_meas[i];
            if (pt_data.if_valid) {
                Eigen::Matrix<double, 1, XSIZE> Hi = Eigen::Matrix<double, 1, XSIZE>::Zero();
                Hi.template leftCols<24>() = pt_data.H;
                double lid_cov = Hi*cov_rcp*Hi.transpose() + pt_data.var_pt;
                const double noise_scale =
                    resple::range::rangeNoiseWeight(rn_cfg, pt_data.range_sensor, rn_norm);
                const bool within_thresh = abs(pt_data.zp) < pt_thresh;
                if (within_thresh || lid_cov < pt_data.var_pt*cov_thresh) {
                    innv_buf_(idx_offset) = - pt_data.zp;
                    H_buf_.row(idx_offset) = Hi;
                    ++accepted_rows;
                    // Instrumentation for the inert-gate finding: rows the
                    // nn_thresh test would have rejected, admitted only by the
                    // cov_thresh disjunct. Counted at admit time, i.e. this
                    // measures the LEGACY exposure regardless of what the
                    // Mahalanobis gate does with the row below.
                    if (!within_thresh) {
                        cov_escape_admits_.fetch_add(1, std::memory_order_relaxed);
                    }
                    const bool reject = lidar_gate.rejects(pt_data.zp, lid_cov);
                    if (reject) { mahal_reject_.push_back(idx_offset); }
                    if (loc_gate_trans_min_eig > 0.0) {
                        // Weight E_tt by the row's RELATIVE information weight
                        // (1/noise_scale): with the close-range down-weighting
                        // active, an unweighted E_tt reads well-constrained
                        // geometry while the weighted information matrix the
                        // filter actually uses is starved — exactly the narrow
                        // -tunnel case the gate exists for. Far points
                        // (weight 1) reproduce the old accumulation exactly.
                        const double w = 1.0 / noise_scale;
                        const Eigen::Matrix3d nnt =
                            pt_data.normvec * pt_data.normvec.transpose();
                        // Gate-flagged rows accumulate separately and are added
                        // back only if the gate disarms — so the armed case never
                        // pays a subtractive cancellation.
                        if (reject) {
                            mahal_Ett.noalias() += w * nnt;
                            mahal_w += w;
                            mahal_n++;
                        } else {
                            gate_Ett.noalias() += w * nnt;
                            gate_w += w;
                            gate_n++;
                        }
                    }
                }
                cov_inv_buf_(idx_offset) =
                    robustWeight(pt_data.zp) / (pt_data.var_pt * noise_scale);
                idx_offset++;
            }
        }
        applyMahalanobisGate(accepted_rows, mahal_Ett, mahal_w, mahal_n,
                             gate_Ett, gate_w, gate_n);
        armLocGate(gate_Ett, gate_w, gate_n);
        return update(innv_buf_, cov_inv_buf_, H_buf_, x_prop, P_prop);
    }

    bool updateLiDARInertial(Eigen::aligned_deque<PointData>& pt_meas, Eigen::aligned_deque<ImuData>& imu_meas, int num_valid, const Eigen::Matrix<double, XSIZE, 1>& x_prop,
        const Eigen::Matrix<double, XSIZE, XSIZE>& P_prop, const double pt_thresh, const double cov_thresh, const Eigen::Vector3d& g, const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyro, int num_threads = 5)
    {
        Eigen::Matrix<double, 6, 1> cov_imu_inv =  Eigen::Matrix<double, 6, 1>(1/cov_acc[0], 1/cov_acc[1], 1/cov_acc[2], 1/cov_gyro[0], 1/cov_gyro[1], 1/cov_gyro[2]);
        #pragma omp parallel for num_threads(num_threads) schedule(dynamic)
        for (size_t i = 0; i < pt_meas.size(); i++) {
            PointData& pt_data = pt_meas[i];
            prepLiDAR(pt_data); 
        }
        #pragma omp parallel for num_threads(num_threads) 
        for (size_t i = 0; i < imu_meas.size(); i++) {
            prepIMU(imu_meas[i], g);
        }
        int dim_meas = 6*imu_meas.size() + num_valid;
        // resize, not conservativeResize — see updateLiDAR: the three buffers
        // are setZero'd immediately below, so the preserved contents are
        // copied and then thrown away.
        H_buf_.resize(dim_meas, XSIZE);
        innv_buf_.resize(dim_meas, 1);
        cov_inv_buf_.resize(dim_meas, 1);
        H_buf_.setZero();
        innv_buf_.setZero();
        cov_inv_buf_.setZero();
        int idx_offset = 0;
        size_t id_imu = 0;
        size_t id_pt = 0;
        // See updateLiDAR: per-update close-range reference (1.0 in the
        // default absolute mode, so the arithmetic below is unchanged).
        const resple::range::RangeNoiseConfig rn_cfg = rangeNoiseConfig();
        const double rn_norm =
            resple::range::rangeNoiseNormalizer(rn_cfg, maxValidRange(pt_meas));
        Eigen::Matrix3d gate_Ett = Eigen::Matrix3d::Zero();
        double gate_w = 0.0;
        int gate_n = 0;
        // LiDAR and IMU rows interleave by timestamp below; the gate needs to
        // know which gain columns belong to LiDAR (IMU keeps full authority).
        if (loc_gate_trans_min_eig > 0.0) {
            loc_gate_mask_ = Eigen::ArrayXd::Zero(dim_meas);
        }
        // Adaptive outlier gate — see updateLiDAR for the mechanism. IMU rows are
        // never gated (they carry their own residual clamp below).
        mahal_reject_.clear();
        Eigen::Matrix3d mahal_Ett = Eigen::Matrix3d::Zero();
        double mahal_w = 0.0;
        int mahal_n = 0;
        int accepted_rows = 0;
        // Hoisted out of the loop below, where it materialized a 4608-byte
        // copy for EVERY valid LiDAR point purely to form a 1x1 quadratic
        // form. cov_rcp is not written anywhere in this function, so the copy
        // is loop-invariant, and the compiler does not hoist it (measured:
        // 449.9 -> 423.0 us on a 300-point call).
        //
        // Kept as a contiguous copy rather than switched to the block
        // expression: the contiguous 24x24 makes the following product faster
        // (loop 68.2 shipped -> 60.1 block-expr -> 57.1 us hoisted, no-BLAS),
        // and a strided block would change the summation order and hence the
        // last ULP of lid_cov, which feeds both the accept test and the
        // hazard-97 gate. This keeps the arithmetic bit-identical.
        const Eigen::Matrix<double, 24, 24> cov =
            cov_rcp.template topLeftCorner<24, 24>();
        for (size_t j = 0; j < imu_meas.size() + pt_meas.size(); j++) {
            if ((id_pt < pt_meas.size() && id_imu < imu_meas.size() && pt_meas[id_pt].time_ns < imu_meas[id_imu].time_ns) ||
                (id_pt < pt_meas.size() && id_imu >= imu_meas.size())) {
                    PointData& pt_data = pt_meas[id_pt];
                    if (pt_data.if_valid) {
                        double lid_cov = pt_data.H*cov*pt_data.H.transpose() + pt_data.var_pt;
                        const double noise_scale =
                            resple::range::rangeNoiseWeight(rn_cfg, pt_data.range_sensor, rn_norm);
                        const bool within_thresh = abs(pt_data.zp) < pt_thresh;
                        if (within_thresh || lid_cov < pt_data.var_pt*cov_thresh) {
                            innv_buf_(idx_offset) = - pt_data.zp;
                            H_buf_.block(idx_offset, 0, 1, 24) = pt_data.H;
                            ++accepted_rows;
                            if (!within_thresh) {
                                cov_escape_admits_.fetch_add(1, std::memory_order_relaxed);
                            }
                            const bool reject = lidar_gate.rejects(pt_data.zp, lid_cov);
                            if (reject) { mahal_reject_.push_back(idx_offset); }
                            if (loc_gate_trans_min_eig > 0.0) {
                                loc_gate_mask_(idx_offset) = 1.0;
                                // Relative information weight — see updateLiDAR.
                                const double w = 1.0 / noise_scale;
                                const Eigen::Matrix3d nnt =
                                    pt_data.normvec * pt_data.normvec.transpose();
                                if (reject) {
                                    mahal_Ett.noalias() += w * nnt;
                                    mahal_w += w;
                                    mahal_n++;
                                } else {
                                    gate_Ett.noalias() += w * nnt;
                                    gate_w += w;
                                    gate_n++;
                                }
                            }
                        }
                        cov_inv_buf_(idx_offset) =
                            robustWeight(pt_data.zp) / (pt_data.var_pt * noise_scale);
                        idx_offset++;
                    }
                    id_pt++;
            } else if ((id_pt < pt_meas.size() && id_imu < imu_meas.size() && pt_meas[id_pt].time_ns >= imu_meas[id_imu].time_ns) ||
                        (id_pt >= pt_meas.size() && id_imu < imu_meas.size())) {
                    const ImuData& imu_data = imu_meas[id_imu];
                    Eigen::Matrix<double, 6, 1> imu_itp = imu_data.imu_itp;
                    Eigen::Matrix<double, 6, XSIZE> Hi = Eigen::Matrix<double, 6, XSIZE>::Zero();
                    Hi.template leftCols<24>() = imu_data.H;
                    Hi.block(0, BA_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
                    Hi.block(3, BG_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();   

                    Eigen::Matrix<double, 6, 1> imu;
                    imu.head<3>() = imu_data.accel;
                    imu.tail<3>() = imu_data.gyro;
                    for (int i = 0; i < 3; i++) {
                        if (abs(imu(i) - imu_itp(i)) > 10.0) {
                            imu(i) = 0;
                            imu_itp(i) = 0;
                            Hi.row(i).setZero();
                        } 
                        if (abs(imu(i+3) - imu_itp(i+3)) > 5.0) {
                            imu(i+3) = 0;
                            imu_itp(i+3) = 0;
                            Hi.row(i+3).setZero();
                        }                     
                    }
                    innv_buf_.segment<6>(idx_offset) = imu - imu_itp;
                    H_buf_.block(idx_offset, 0, 6, XSIZE) = Hi;
                    cov_inv_buf_.segment<6>(idx_offset) = cov_imu_inv;  
                    idx_offset += 6;
                    id_imu++;
            }
        }
        applyMahalanobisGate(accepted_rows, mahal_Ett, mahal_w, mahal_n,
                             gate_Ett, gate_w, gate_n);
        armLocGate(gate_Ett, gate_w, gate_n);
        return update(innv_buf_, cov_inv_buf_, H_buf_, x_prop, P_prop);
    }

    // Returns false if the update was skipped due to numerical failure.
    template <int RSIZE>
    bool update(const Eigen::Matrix<double, RSIZE, 1>& innov, const Eigen::Matrix<double, RSIZE, 1>& R_inv, const Eigen::Matrix<double, RSIZE, XSIZE>& H,
        const Eigen::Matrix<double, XSIZE, 1>& x_prop, const Eigen::Matrix<double, XSIZE, XSIZE>& cov_prop)
    {
        int num_pts = innov.rows();
        // Pessimistic default: any early (failure) exit leaves NIS = NaN so the
        // divergence detector counts the skipped update as a breach.
        // dof counts only rows carrying information: the assemblers emit
        // all-zero rows for gate-rejected points and outlier-clamped IMU axes
        // (H row and innovation both zeroed). Those rows contribute 0 to the
        // NIS sum, so counting them in dof deflates NIS/dof and desensitizes
        // the §3.3 divergence detector exactly when many measurements are
        // being rejected.
        int dof = 0;
        for (int i = 0; i < num_pts; i++) {
            if (innov(i) != 0.0 || (H.row(i).array() != 0.0).any()) dof++;
        }
        last_nis_dof_ = dof;
        last_nis_ = std::numeric_limits<double>::quiet_NaN();
        Eigen::Matrix<double, XSIZE, 1> RCPs_post;
        Eigen::MatrixXd I_X = Eigen::MatrixXd::Identity(XSIZE, XSIZE);
        if (num_pts > XSIZE) {
            auto llt_prop = cov_prop.llt();
            if (llt_prop.info() != Eigen::Success) {
                std::cerr << "[Estimator] cov_prop LLT decomposition failed, skipping update\n";
                return false;
            }
            Eigen::Matrix<double, XSIZE, XSIZE> cov_rcp_inv = llt_prop.solve(I_X);
            Eigen::Matrix<double, XSIZE, RSIZE> HT_R_inv;
            HT_R_inv.noalias() = (H.transpose().array().rowwise() * R_inv.transpose().array()).matrix();
            Eigen::Matrix<double, XSIZE, XSIZE> HT_R_inv_H;
            HT_R_inv_H.noalias() = HT_R_inv * H;

            Eigen::Matrix<double, XSIZE, XSIZE> S = HT_R_inv_H;
            S.noalias() += cov_rcp_inv;
            auto llt_S = S.llt();
            if (llt_S.info() != Eigen::Success) {
                std::cerr << "[Estimator] S LLT decomposition failed, skipping update\n";
                return false;
            }
            Eigen::Matrix<double, XSIZE, XSIZE> S_inv = llt_S.solve(I_X);
            Eigen::Matrix<double, XSIZE, RSIZE> K;
            K.noalias() = S_inv * HT_R_inv;

            // Measurement-space NIS via the Woodbury identity: with
            // S = P^{-1} + H^T R^{-1} H (the state-space matrix factored above),
            //   nu^T (H P H^T + R)^{-1} nu = nu^T R^{-1} nu - b^T S^{-1} b,
            // where b = H^T R^{-1} nu. Reuses the existing Cholesky factor, so
            // the extra cost is one XSIZE-vector solve and two dot products.
            {
                const Eigen::Matrix<double, RSIZE, 1> Rinv_nu = R_inv.cwiseProduct(innov);
                const Eigen::Matrix<double, XSIZE, 1> b = H.transpose() * Rinv_nu;
                last_nis_ = innov.dot(Rinv_nu) - b.dot(llt_S.solve(b));
            }

            // Degenerate-direction gate: project the LiDAR columns of K out of
            // the unobservable translation axes, then KH must be rebuilt from
            // the projected gain (Joseph form below stays valid for any gain).
            if (loc_gate_armed_ && loc_gate_mask_.size() == K.cols()) {
                applyLocGate(K);
                KH.noalias() = K * H;
            } else {
                KH.noalias() = S_inv * HT_R_inv_H;
            }
            loc_gate_armed_ = false;
            Eigen::Matrix<double, XSIZE, 1> delta_cur = (getState() - x_prop);
            Eigen::Matrix<double, XSIZE, 1> deltax = KH * delta_cur + K * innov - delta_cur;
            RCPs_post.noalias() = getState() + deltax;

            // Joseph-form posterior covariance: P_post = (I - KH) P (I - KH)^T + K R K^T
            // R is diagonal with entries 1/R_inv. Scaling K's columns by 1/R_inv gives K*R.
            Eigen::Matrix<double, XSIZE, XSIZE> I_KH = Eigen::Matrix<double, XSIZE, XSIZE>::Identity() - KH;
            cov_post_.noalias() = I_KH * cov_prop * I_KH.transpose();
            Eigen::Matrix<double, XSIZE, RSIZE> KR =
                (K.array().rowwise() * R_inv.transpose().array().inverse()).matrix();
            cov_post_.noalias() += KR * K.transpose();
        } else {
            Eigen::Matrix<double, RSIZE, RSIZE> R = R_inv.cwiseInverse().asDiagonal();
            Eigen::Matrix<double, RSIZE, RSIZE> S;
            S.noalias() = H * cov_prop * H.transpose() + R;
            Eigen::FullPivLU<Eigen::Matrix<double, RSIZE, RSIZE>> lu_S(S);
            if (!lu_S.isInvertible()) {
                std::cerr << "[Estimator] S matrix singular, skipping update\n";
                return false;
            }
            Eigen::Matrix<double, XSIZE, RSIZE> K;
            K.noalias() = cov_prop * H.transpose() * lu_S.inverse();
            // Degenerate-direction gate (see information-form branch above).
            if (loc_gate_armed_ && loc_gate_mask_.size() == K.cols()) {
                applyLocGate(K);
            }
            loc_gate_armed_ = false;
            KH.noalias() = K * H;
            // Measurement-space NIS directly from S = H P H^T + R (already
            // factored for the gain): nu^T S^{-1} nu.
            last_nis_ = innov.dot(lu_S.solve(innov));
            Eigen::Matrix<double, XSIZE, 1> delta_cur = (getState() - x_prop);
            Eigen::Matrix<double, XSIZE, 1> deltax = KH * delta_cur + K * innov - delta_cur;
            RCPs_post.noalias() = getState() + deltax;

            // Joseph-form posterior covariance: P_post = (I - KH) P (I - KH)^T + K R K^T
            Eigen::Matrix<double, XSIZE, XSIZE> I_KH = Eigen::Matrix<double, XSIZE, XSIZE>::Identity() - KH;
            cov_post_.noalias() = I_KH * cov_prop * I_KH.transpose();
            Eigen::Matrix<double, XSIZE, RSIZE> KR =
                (K.array().rowwise() * R_inv.transpose().array().inverse()).matrix();
            cov_post_.noalias() += KR * K.transpose();
        }
        // Guard against NaN/Inf reaching the spline / cov_rcp. The LLT checks
        // above validate the *inputs* (cov_prop, S); this catches a non-finite
        // *result* (e.g. a zero R_inv entry making K*R = inf*0 = NaN in the
        // Joseph K R K^T term). Returning false routes the caller to its
        // reset-to-prior + numerical-failure-counter path.
        if (!cov_post_.allFinite() || !RCPs_post.allFinite()) {
            std::cerr << "[Estimator] non-finite posterior/state update, skipping\n";
            // A rejected update is a consistency breach regardless of the NIS
            // value computed mid-branch — restore the NaN sentinel.
            last_nis_ = std::numeric_limits<double>::quiet_NaN();
            return false;
        }
        updateState(RCPs_post);
        return true;
    }
};
