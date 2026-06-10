#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include "utils/common_utils.h"
#include "SplineState.h"
#include "ikd-Tree/ikd_Tree.h"

#ifdef RESPLE_USE_CUDA
#include "gpu/cuda_knn.h"
#endif

class Association
{
public:
    // Diagnostic counter: how many pointBodyToWorld queries hit a timestamp
    // outside the current spline support [minTimeNs, maxTimeNs]. findCorresp
    // has an active-window guard, but RESPLE::processData's deskew loop calls
    // pointBodyToWorld on every pt_meas entry (including ones findCorresp
    // skipped) — that path has no window check, so a stale or out-of-order
    // scan can silently extrapolate. This counter surfaces that in
    // updateDiagnostics. Inline-static so the single definition is shared
    // across TUs (C++17).
    inline static std::atomic<uint64_t> out_of_range_queries_{0};

    // HARDENING §3.2: correspondence thresholds, previously hardcoded in
    // findCorresp. Defaults reproduce the legacy values bit-for-bit
    // (max_neighbor_sq_dist 5 == the old Nearest_Search 2.236² gate,
    // plane_fit_thresh 0.1, no degeneracy guard).
    struct CorrespConfig {
        // Reject a candidate unless all k neighbors lie within this squared
        // distance (m²) of the query point. NOTE: the CUDA k-NN's voxel-shell
        // early termination (gpu/cuda_knn) covers 2.4 m and was validated
        // against the default 2.236 m (= sqrt 5) gate — raising this above
        // ~5.76 m² under ENABLE_CUDA requires widening that scan first.
        float max_neighbor_sq_dist = 5.0f;
        // esti_plane residual threshold (m): every neighbor must lie within
        // this distance of the fitted plane.
        float plane_fit_thresh = 0.1f;
        // Rank-conditioning guard forwarded to resple::geom::fitPlane: relative
        // QR pivot threshold rejecting degenerate (collinear / rank-deficient)
        // neighbor patches that lie on infinitely many planes. 0 disables
        // (legacy behaviour); see geometry_core.h for semantics.
        float plane_min_cond_ratio = 0.0f;
    };

    // HARDENING §3.2: per-update correspondence funnel. Each stage counts
    // points surviving the previous gate, so stage-to-stage drops show WHERE
    // candidates are lost (out of window / sparse map / degenerate or noisy
    // patch / far from plane). Accumulated across the IEKF's inner iterations
    // of one update (n_iter is 1 in the shipping config, so per-scan in
    // practice).
    struct CorrespStats {
        uint32_t candidates = 0;     // points fed to findCorresp
        uint32_t passed_window = 0;  // inside the active 4-knot window (k-NN ran)
        uint32_t passed_knn = 0;     // full k neighbors within max_neighbor_sq_dist
        uint32_t passed_plane = 0;   // esti_plane accepted the neighbor patch
        uint32_t used = 0;           // final if_valid (point-to-plane range gate)
        void reset() { *this = CorrespStats{}; }
    };

    template<class PointType>
    static void pointBodyToWorld(int64_t t_ns, const SplineState* spline, const PointType& pi, PointType& po, const Eigen::Vector3d& t_bl, const Eigen::Quaterniond& q_bl)
    {
        // Clamp t_ns to the spline's active support before calling itpPose.
        //
        // The Phase-1 fix logged out-of-range queries but still passed the
        // bad t_ns through — itpPose's interior knot loop indexes
        // t_knots[idx0 + i] without a bound check (SplineState.h around
        // line 393), and for t_ns > maxTimeNs() that index can equal or
        // exceed num_knot, OOB-reading the deque → SIGSEGV. Clamping here
        // converts a deterministic crash into a deterministic boundary
        // evaluation: the deskew uses the pose at the spline edge instead
        // of an extrapolated one. Geometrically slightly off, never UB.
        // The counter still increments so operators can see when scan
        // timestamps drift outside the active window.
        const int64_t t_min = spline->minTimeNs();
        const int64_t t_max = spline->maxTimeNs();
        const int64_t t_ns_safe = std::clamp(t_ns, t_min, t_max);
        if (t_ns_safe != t_ns) {
            if (out_of_range_queries_.fetch_add(1, std::memory_order_relaxed) == 0) {
                std::cerr << "[Association] pointBodyToWorld: t_ns=" << t_ns
                          << " outside spline range [" << t_min << "," << t_max
                          << "]; clamping to " << t_ns_safe
                          << ". Further occurrences silent, see diagnostics counter.\n";
            }
        }
        Eigen::Quaterniond q;
        Eigen::Vector3d pos;
        spline->itpPose(t_ns_safe, &pos, nullptr, &q, nullptr);
        Eigen::Vector3f p_body(pi.x, pi.y, pi.z);
        Eigen::Vector3f p_global = q.cast<float>() * (q_bl.cast<float>() * p_body + t_bl.cast<float>()) + pos.cast<float>();
        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.curvature = pi.curvature;
    }

    // CPU path: per-point ikd-tree search (current production path).
    static void findCorresp(int& effect_num_k, const SplineState* spline, KD_TREE<pcl::PointXYZINormal>* ikdtree,
                            Eigen::aligned_deque<PointData>& pt_meas,
                            std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
                            const CorrespConfig& cfg, CorrespStats* stats,
                            int num_threads = 5, int num_match_points = 5)
    {
        int num_pt = pt_meas.size();
        const float max_nn_dist = std::sqrt(cfg.max_neighbor_sq_dist);
        if (stats) stats->candidates += num_pt;
        // schedule(static): each thread writes only to its own pt_meas[i] / pt_neighbors[i], no synchronization needed.
        #pragma omp parallel num_threads(num_threads)
        {
        // Thread-local scratch buffer reused across iterations to avoid per-point heap allocation.
        std::vector<float> pointSearchSqDis;
        pointSearchSqDis.reserve(num_match_points);
        // Thread-local stage counters, merged once per thread below — no
        // per-point synchronization on the hot path.
        uint32_t n_window = 0, n_knn = 0, n_plane = 0;
        #pragma omp for schedule(static) nowait
        for (int i = 0; i < num_pt; i++) {
            PointData& pt_data = pt_meas[i];
            pt_data.if_valid = false;
            // At startup (exactly 4 knots), accept all points. Once the spline
            // has grown, only accept points within the active 4-knot window.
            const bool in_active_window = pt_data.time_ns <= spline->maxTimeNs()
                                       && pt_data.time_ns >= spline->maxTimeNs() - 4 * spline->getKnotTimeIntervalNs();
            if (spline->numKnots() != 4 && !in_active_window) {
                pt_neighbors[i].clear();
                continue;
            }
            n_window++;
            Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w, pt_data.t_bl, pt_data.q_bl);
            pt_neighbors[i].clear();
            ikdtree->Nearest_Search(pt_data.pt_w, num_match_points, pt_neighbors[i], pointSearchSqDis, max_nn_dist);
            if (pt_neighbors[i].size() >= (size_t)num_match_points && pointSearchSqDis[num_match_points - 1] < cfg.max_neighbor_sq_dist) {
                n_knn++;
                Eigen::Vector4f pabcd;
                pabcd.setZero();
                if (CommonUtils::esti_plane(pabcd, pt_neighbors[i], cfg.plane_fit_thresh, cfg.plane_min_cond_ratio)) {
                    n_plane++;
                    float pd2 = pabcd(0) * pt_data.pt_w.x + pabcd(1) * pt_data.pt_w.y + pabcd(2) * pt_data.pt_w.z + pabcd(3);
                    if (pt_data.range_sensor > 81.0 * pd2 * pd2) {
                        pt_data.if_valid = true;
                        pt_data.normvec = Eigen::Vector3d(pabcd[0], pabcd[1], pabcd[2]);
                        pt_data.dist = pabcd(3);
                    }
                }
            }
        }
        if (stats) {
            #pragma omp atomic
            stats->passed_window += n_window;
            #pragma omp atomic
            stats->passed_knn += n_knn;
            #pragma omp atomic
            stats->passed_plane += n_plane;
        }
        }  // end omp parallel
        for (int i = 0; i < num_pt; i++) {
            if (pt_meas[i].if_valid) {
                effect_num_k++;
                if (stats) stats->used++;
            }
        }
    }

#ifdef RESPLE_USE_CUDA
    // GPU path: three-phase batched correspondence finding.
    //   Phase 1 (parallel CPU): body→world transform + active-window check.
    //   Phase 2 (single GPU launch): batch k-NN over all queries.
    //   Phase 3 (parallel CPU): plane fit + outlier validation.
    static void findCorresp(int& effect_num_k, const SplineState* spline,
                            resple_gpu::CudaMap* cuda_map,
                            Eigen::aligned_deque<PointData>& pt_meas,
                            std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>>& pt_neighbors,
                            const CorrespConfig& cfg, CorrespStats* stats,
                            int num_threads = 5, int num_match_points = 5)
    {
        const int num_pt = static_cast<int>(pt_meas.size());
        if (stats) stats->candidates += num_pt;

        // Phase 1: body→world + active-window mask.
        // Use a sentinel point (NaN) for inactive queries so the GPU still
        // processes them in lockstep but their results are guaranteed to
        // miss the validation thresholds in phase 3.
        std::vector<pcl::PointXYZINormal> queries(num_pt);
        std::vector<char> active(num_pt, 0);
        #pragma omp parallel for num_threads(num_threads) schedule(static)
        for (int i = 0; i < num_pt; i++) {
            PointData& pt_data = pt_meas[i];
            pt_data.if_valid = false;
            const bool in_active_window = pt_data.time_ns <= spline->maxTimeNs()
                                       && pt_data.time_ns >= spline->maxTimeNs() - 4 * spline->getKnotTimeIntervalNs();
            if (spline->numKnots() != 4 && !in_active_window) {
                pt_neighbors[i].clear();
                queries[i].x = std::numeric_limits<float>::quiet_NaN();
                queries[i].y = std::numeric_limits<float>::quiet_NaN();
                queries[i].z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w,
                                          pt_data.t_bl, pt_data.q_bl);
            queries[i] = pt_data.pt_w;
            active[i] = 1;
        }

        // Phase 2: single GPU kernel launch for all queries.
        std::vector<pcl::PointXYZINormal> nbrs;
        std::vector<float> dist_sq;
        cuda_map->batch_search(queries.data(), queries.size(),
                               num_match_points, nbrs, dist_sq);

        // Phase 3: parallel plane fit + validation.
        #pragma omp parallel num_threads(num_threads)
        {
        uint32_t n_knn = 0, n_plane = 0;
        #pragma omp for schedule(static) nowait
        for (int i = 0; i < num_pt; i++) {
            if (!active[i]) continue;
            PointData& pt_data = pt_meas[i];

            // Gather neighbor points from the kernel output (no host
            // mirror lookup — coords are already in nbrs).
            pt_neighbors[i].clear();
            pt_neighbors[i].reserve(num_match_points);
            bool ok = true;
            for (int j = 0; j < num_match_points; j++) {
                if (!std::isfinite(dist_sq[i * num_match_points + j])) {
                    ok = false; break;
                }
                pt_neighbors[i].push_back(nbrs[i * num_match_points + j]);
            }
            if (!ok || pt_neighbors[i].size() < (size_t)num_match_points) continue;
            if (dist_sq[i * num_match_points + num_match_points - 1] >= cfg.max_neighbor_sq_dist) continue;
            n_knn++;

            Eigen::Vector4f pabcd;
            pabcd.setZero();
            if (CommonUtils::esti_plane(pabcd, pt_neighbors[i], cfg.plane_fit_thresh, cfg.plane_min_cond_ratio)) {
                n_plane++;
                float pd2 = pabcd(0) * pt_data.pt_w.x + pabcd(1) * pt_data.pt_w.y
                          + pabcd(2) * pt_data.pt_w.z + pabcd(3);
                if (pt_data.range_sensor > 81.0 * pd2 * pd2) {
                    pt_data.if_valid = true;
                    pt_data.normvec = Eigen::Vector3d(pabcd[0], pabcd[1], pabcd[2]);
                    pt_data.dist = pabcd(3);
                }
            }
        }
        if (stats) {
            #pragma omp atomic
            stats->passed_knn += n_knn;
            #pragma omp atomic
            stats->passed_plane += n_plane;
        }
        }  // end omp parallel

        for (int i = 0; i < num_pt; i++) {
            if (stats && active[i]) stats->passed_window++;
            if (pt_meas[i].if_valid) {
                effect_num_k++;
                if (stats) stats->used++;
            }
        }
    }
#endif  // RESPLE_USE_CUDA
};
