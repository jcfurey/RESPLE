#pragma once

#include <cstddef>
#include <vector>
#include <pcl/point_types.h>

namespace resple_gpu {

// CudaMap: GPU-resident point cloud + brute-force batched k-NN.
//
// Lifecycle:
//   1) RESPLE.cpp owns one instance.
//   2) After each mapIncremental, RESPLE flattens the ikd-tree to a vector
//      and calls update() to sync the GPU array.
//   3) findCorresp collects all per-IEKF-iteration queries into a single
//      vector and calls batch_search() once.
//
// Thread-safety:
//   Not internally synchronized. The caller (RESPLE.cpp) coordinates
//   update() vs batch_search() via the existing mtx_map_ shared_mutex
//   that already guards the underlying ikd-tree.
//
// Build:
//   When the package is built with -DENABLE_CUDA=ON, RESPLE_USE_CUDA is
//   defined and this class delegates to the CUDA implementation in
//   cuda_knn.cu. When built without CUDA, the constructor throws — the
//   call sites are #ifdef-guarded so the class is never instantiated in
//   that configuration.
class CudaMap {
public:
    CudaMap();
    ~CudaMap();

    CudaMap(const CudaMap&) = delete;
    CudaMap& operator=(const CudaMap&) = delete;

    // Replace the GPU map with the given points. Resizes if needed.
    // Allocator-agnostic: takes a raw pointer + count so callers can pass
    // either std::vector or Eigen::aligned_vector storage.
    void update(const pcl::PointXYZINormal* points, std::size_t n);

    // Batch k-NN: for each query, find the k nearest map points.
    //
    // Output neighbors: n_queries * k entries, row-major (PCL points).
    //   Entry [i*k + j] is the j-th nearest neighbor of query i.
    //   x/y/z/intensity are valid; normals/curvature are zeroed.
    //   For positions where fewer than k neighbors exist, dist_sq[..]
    //   will be INFINITY and the point coordinates are undefined.
    // Output dist_sq: same shape, squared Euclidean distances.
    //
    // Returning neighbor coordinates directly (vs indices) lets us skip
    // the expensive DtoH copy of the entire map at update() time and
    // the host-side mirror — important for keeping per-map-update
    // latency low at high playback rates.
    void batch_search(const pcl::PointXYZINormal* queries, std::size_t n_queries,
                      int k,
                      std::vector<pcl::PointXYZINormal>& neighbors,
                      std::vector<float>& dist_sq);

    std::size_t n_points() const;
    bool empty() const;

    // Drop the GPU-resident map (n_points → 0, so empty()==true) without
    // freeing device buffers. Call on lifecycle deactivate so a re-activated
    // node never queries the previous run's stale map before the next update().
    // If a CUDA op has failed (sticky), empty() stays true regardless.
    void clear();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};

}  // namespace resple_gpu
