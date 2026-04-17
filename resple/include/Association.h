#pragma once

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

    template<class PointType>
    static void pointBodyToWorld(int64_t t_ns, const SplineState* spline, const PointType& pi, PointType& po, const Eigen::Vector3d& t_bl, const Eigen::Quaterniond& q_bl)
    {
        Eigen::Quaterniond q;
        Eigen::Vector3d pos;
        spline->itpPose(t_ns, &pos, nullptr, &q, nullptr);
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
                            int num_threads = 5, int num_match_points = 5)
    {
        int num_pt = pt_meas.size();
        // schedule(static): each thread writes only to its own pt_meas[i] / pt_neighbors[i], no synchronization needed.
        #pragma omp parallel num_threads(num_threads)
        {
        // Thread-local scratch buffer reused across iterations to avoid per-point heap allocation.
        std::vector<float> pointSearchSqDis;
        pointSearchSqDis.reserve(num_match_points);
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
            Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w, pt_data.t_bl, pt_data.q_bl);
            pt_neighbors[i].clear();
            ikdtree->Nearest_Search(pt_data.pt_w, num_match_points, pt_neighbors[i], pointSearchSqDis, 2.236);
            if (pt_neighbors[i].size() >= (size_t)num_match_points && pointSearchSqDis[num_match_points - 1] < 5) {
                Eigen::Vector4f pabcd;
                pabcd.setZero();
                if (CommonUtils::esti_plane(pabcd, pt_neighbors[i], 0.1f)) {
                    float pd2 = pabcd(0) * pt_data.pt_w.x + pabcd(1) * pt_data.pt_w.y + pabcd(2) * pt_data.pt_w.z + pabcd(3);
                    if (pt_data.range_sensor > 81.0 * pd2 * pd2) {
                        pt_data.if_valid = true;
                        pt_data.normvec = Eigen::Vector3d(pabcd[0], pabcd[1], pabcd[2]);
                        pt_data.dist = pabcd(3);
                    }
                }
            }
        }
        }  // end omp parallel
        for (int i = 0; i < num_pt; i++) {
            if (pt_meas[i].if_valid) {
                effect_num_k++;
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
                            int num_threads = 5, int num_match_points = 5)
    {
        const int num_pt = static_cast<int>(pt_meas.size());

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
        #pragma omp parallel for num_threads(num_threads) schedule(static)
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
            if (dist_sq[i * num_match_points + num_match_points - 1] >= 5) continue;

            Eigen::Vector4f pabcd;
            pabcd.setZero();
            if (CommonUtils::esti_plane(pabcd, pt_neighbors[i], 0.1f)) {
                float pd2 = pabcd(0) * pt_data.pt_w.x + pabcd(1) * pt_data.pt_w.y
                          + pabcd(2) * pt_data.pt_w.z + pabcd(3);
                if (pt_data.range_sensor > 81.0 * pd2 * pd2) {
                    pt_data.if_valid = true;
                    pt_data.normvec = Eigen::Vector3d(pabcd[0], pabcd[1], pabcd[2]);
                    pt_data.dist = pabcd(3);
                }
            }
        }

        for (int i = 0; i < num_pt; i++) {
            if (pt_meas[i].if_valid) effect_num_k++;
        }
    }
#endif  // RESPLE_USE_CUDA
};
