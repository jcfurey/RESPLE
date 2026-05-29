#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include "SplineState.h"
#include "Association.h"

template<int XSIZE>
class Estimator
{
  public:
    static const int CP_SIZE = 24;
    static const int BA_OFFSET = 24;
    static const int BG_OFFSET = 27;
    int n_iter = 1;

    // Atomic because updateDiagnostics may read these from the ROS executor
    // thread while the worker is mid-IEKF. Monotonic; no reset — the diagnostics
    // publisher snapshots deltas.
    std::atomic<uint64_t> num_numerical_failures_{0};

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
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, ikdtree, pt_meas, pt_neighbors, num_threads, num_match_points);
            }
            if (num_tot_eff > 0) {
                if (!updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads)) {
                    std::cerr << "[Estimator] IEKF LiDAR update failed (numerical), resetting covariance to prior\n";
                    num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                    cov_rcp = cov_prop;
                    break;
                }
            } else {
                break;
            }
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
    }

#ifdef RESPLE_USE_CUDA
    // GPU overload: same logic, but findCorresp uses the batched CudaMap path.
    void updateIEKFLiDAR(Eigen::aligned_deque<PointData>& pt_meas,
                         std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
                         resple_gpu::CudaMap* cuda_map, const double pt_thresh, const double cov_thresh,
                         int num_threads = 5, int num_match_points = 5)
    {
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, cuda_map, pt_meas, pt_neighbors, num_threads, num_match_points);
            }
            if (num_tot_eff > 0) {
                if (!updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads)) {
                    std::cerr << "[Estimator] IEKF LiDAR update failed (numerical), resetting covariance to prior\n";
                    num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                    cov_rcp = cov_prop;
                    break;
                }
            } else {
                break;
            }
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
    }

    void updateIEKFLiDARInertial(Eigen::aligned_deque<PointData>& pt_meas,
        std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
        resple_gpu::CudaMap* cuda_map, const double pt_thresh,
        Eigen::aligned_deque<ImuData>& imu_meas, const Eigen::Vector3d& g, const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyro, const double cov_thresh,
        int num_threads = 5, int num_match_points = 5)
    {
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, cuda_map, pt_meas, pt_neighbors, num_threads, num_match_points);
            }
            bool update_ok = false;
            if (num_tot_eff > 0 && imu_meas.empty()) {
                update_ok = updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads);
            } else if (num_tot_eff > 0) {
                update_ok = updateLiDARInertial(pt_meas, imu_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, g, cov_acc, cov_gyro, num_threads);
            } else {
                break;
            }
            if (!update_ok) {
                std::cerr << "[Estimator] IEKF LIO update failed (numerical), resetting covariance to prior\n";
                num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                cov_rcp = cov_prop;
                break;
            }
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
    }
#endif  // RESPLE_USE_CUDA

    void updateIEKFLiDARInertial(Eigen::aligned_deque<PointData>& pt_meas,
        std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
        KD_TREE<pcl::PointXYZINormal>* ikdtree, const double pt_thresh,
        Eigen::aligned_deque<ImuData>& imu_meas, const Eigen::Vector3d& g, const Eigen::Vector3d& cov_acc, const Eigen::Vector3d& cov_gyro, const double cov_thresh,
        int num_threads = 5, int num_match_points = 5)
    {
        const Eigen::Matrix<double, XSIZE, XSIZE> cov_prop = cov_rcp;
        Eigen::Matrix<double, XSIZE, 1> rcp_prop = getState();
        bool converged = true;
        int num_tot_eff = 0;
        int t = 0;
        for (int i = 0; i < max_iter; i++) {
            Eigen::Matrix<double, XSIZE, 1> rcpi = getState();
            if (converged) {
                num_tot_eff = 0;
                Association::findCorresp(num_tot_eff, &spl, ikdtree, pt_meas, pt_neighbors, num_threads, num_match_points);
            }
            bool update_ok = false;
            if (num_tot_eff > 0 && imu_meas.empty()) {
                update_ok = updateLiDAR(pt_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, num_threads);
            } else if (num_tot_eff > 0) {
                update_ok = updateLiDARInertial(pt_meas, imu_meas, num_tot_eff, rcp_prop, cov_prop, pt_thresh, cov_thresh, g, cov_acc, cov_gyro, num_threads);
            } else {
                break;
            }
            if (!update_ok) {
                std::cerr << "[Estimator] IEKF LIO update failed (numerical), resetting covariance to prior\n";
                num_numerical_failures_.fetch_add(1, std::memory_order_relaxed);
                cov_rcp = cov_prop;
                break;
            }
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
    }

    void propRCP(int64_t t)
    {
        if (spl.maxTimeNs() >= t) {
            cov_rcp += cov_sys;
        } else {
            while (spl.maxTimeNs() < t) {
                Eigen::Matrix<double, 24, 1> cps_win = spl.getRCPs();
                Eigen::Matrix<double, 6, 1> cp_prop_pos = 2*cps_win.block<6, 1>(12, 0) - cps_win.block<6, 1>(0, 0);
                Eigen::Vector3d delta = cps_win.segment<3>(9);
                spl.addOneStateKnot(cp_prop_pos.head<3>(), delta);
                cov_rcp = a_mat * cov_rcp * a_mat.transpose() + cov_sys;
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

        // G (3×4): maps 4D δq [w,x,y,z] → 3D body-frame rotation vector.
        // Derived from 2·imag(q_out⁻¹ ⊗ δq) = 2·[rows 1,2,3 of Qleft(q_out⁻¹)] · δq.
        const double qw = q_out.w(), qx = q_out.x(), qy = q_out.y(), qz = q_out.z();
        Eigen::Matrix<double, 3, 4> G;
        G << -qx,  qw,  qz, -qy,
             -qy, -qz,  qw,  qx,
             -qz,  qy, -qx,  qw;
        G *= 2.0;

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
    SplineState spl;
    // Zero-initialized explicitly: the default workspace build uses
    // RelWithDebInfo which disables Eigen's optional NaN-init (Debug-only
    // EIGEN_INITIALIZE_MATRICES_BY_NAN). Previously these matrices held
    // whatever was on the heap until setState() / update() wrote them; any
    // read before that first write would have been UB. setState() now runs
    // on valid zeros.
    Eigen::Matrix<double, XSIZE, XSIZE> cov_rcp  = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    Eigen::Matrix<double, XSIZE, XSIZE> cov_sys  = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    Eigen::Matrix<double, XSIZE, XSIZE> a_mat    = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, XSIZE, XSIZE> KH       = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    // Joseph-form posterior covariance set by update() each iteration.
    // P_post = (I - KH) P_prior (I - KH)^T + K R K^T -- numerically PSD-preserving
    // in floating point, unlike the simpler (I - KH) P_prior form.
    Eigen::Matrix<double, XSIZE, XSIZE> cov_post_ = Eigen::Matrix<double, XSIZE, XSIZE>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, XSIZE> H_buf_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> innv_buf_;
    Eigen::Matrix<double, Eigen::Dynamic, 1> cov_inv_buf_;
    int max_iter = 5;
    double eps = 0.1;

    void prepIMU(ImuData& imu_data, const Eigen::Vector3d& g)
    {
        Eigen::Quaterniond q_itp;
        Eigen::Vector3d rot_vel;
        Jacobian43 J_ortdel;
        Jacobian J_line_acc;
        Jacobian33 J_gyro;
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
        Hi.block(0, BA_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();
        Hi.block(3, BG_OFFSET, 3, 3) = Eigen::Matrix3d::Identity();            
        imu_data.imu_itp.head<3>() = RT * a_w + ba;
        imu_data.imu_itp.tail<3>() = rot_vel + bg;
        imu_data.H = Hi.template leftCols<24>();
    }    

    void prepLiDAR(PointData& pt_data) const
    {
        if (pt_data.if_valid) {
            Eigen::Matrix<double, 1, XSIZE> Hi = Eigen::Matrix<double, 1, XSIZE>::Zero();
            Eigen::Quaterniond q_itp;
            Jacobian43 J_ortdel;
            Jacobian J_pos;
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
        H_buf_.conservativeResize(num_valid, XSIZE);
        innv_buf_.conservativeResize(num_valid, 1);
        cov_inv_buf_.conservativeResize(num_valid, 1);
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
        for(size_t i = 0; i < num_pt; i++) {
            const PointData& pt_data = pt_meas[i];
            if (pt_data.if_valid) {
                Eigen::Matrix<double, 1, XSIZE> Hi = Eigen::Matrix<double, 1, XSIZE>::Zero();
                Hi.template leftCols<24>() = pt_data.H;
                double lid_cov = Hi*cov_rcp*Hi.transpose() + pt_data.var_pt;
                if (abs(pt_data.zp) < pt_thresh || lid_cov < pt_data.var_pt*cov_thresh) {
                    innv_buf_(idx_offset) = - pt_data.zp;
                    H_buf_.row(idx_offset) = Hi;

                }
                constexpr double range_ref = 3.0;
                double r = std::max(static_cast<double>(pt_data.range_sensor), 0.1);
                double noise_scale = (r < range_ref) ? (range_ref * range_ref) / (r * r) : 1.0;
                cov_inv_buf_(idx_offset) = 1.0 / (pt_data.var_pt * noise_scale);
                idx_offset++;
            }
        }
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
        H_buf_.conservativeResize(dim_meas, XSIZE);
        innv_buf_.conservativeResize(dim_meas, 1);
        cov_inv_buf_.conservativeResize(dim_meas, 1);
        H_buf_.setZero();
        innv_buf_.setZero();
        cov_inv_buf_.setZero();
        int idx_offset = 0;
        size_t id_imu = 0;
        size_t id_pt = 0;
        for (size_t j = 0; j < imu_meas.size() + pt_meas.size(); j++) {
            if ((id_pt < pt_meas.size() && id_imu < imu_meas.size() && pt_meas[id_pt].time_ns < imu_meas[id_imu].time_ns) ||
                (id_pt < pt_meas.size() && id_imu >= imu_meas.size())) {
                    PointData& pt_data = pt_meas[id_pt];
                    if (pt_data.if_valid) {
                        Eigen::Matrix<double, 24, 24> cov = cov_rcp.template topLeftCorner<24, 24>();
                        double lid_cov = pt_data.H*cov*pt_data.H.transpose() + pt_data.var_pt;
                        if (abs(pt_data.zp) < pt_thresh || lid_cov < pt_data.var_pt*cov_thresh) {
                            innv_buf_(idx_offset) = - pt_data.zp;
                            H_buf_.block(idx_offset, 0, 1, 24) = pt_data.H;
                        }
                        constexpr double range_ref = 3.0;
                        double r = std::max(static_cast<double>(pt_data.range_sensor), 0.1);
                        double noise_scale = (r < range_ref) ? (range_ref * range_ref) / (r * r) : 1.0;
                        cov_inv_buf_(idx_offset) = 1.0 / (pt_data.var_pt * noise_scale);
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
        return update(innv_buf_, cov_inv_buf_, H_buf_, x_prop, P_prop);
    }

    // Returns false if the update was skipped due to numerical failure.
    template <int RSIZE>
    bool update(const Eigen::Matrix<double, RSIZE, 1>& innov, const Eigen::Matrix<double, RSIZE, 1>& R_inv, const Eigen::Matrix<double, RSIZE, XSIZE>& H,
        const Eigen::Matrix<double, XSIZE, 1>& x_prop, const Eigen::Matrix<double, XSIZE, XSIZE>& cov_prop)
    {
        int num_pts = innov.rows();
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

            KH.noalias() = S_inv * HT_R_inv_H;
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
            KH.noalias() = K * H;
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
            return false;
        }
        updateState(RCPs_post);
        return true;
    }
};
