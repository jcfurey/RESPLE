#include "gpu/cuda_knn.h"
#include "gpu/cuda_profile.h"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cub/cub.cuh>

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace resple_gpu {

namespace {

constexpr int kMaxK = 8;
constexpr int kThreadsPerBlock = 128;

// Voxel size for hash-grid bucketing. We deliberately keep this larger than
// the map downsample (ds_lm_voxel ~ 0.2-0.5 m) so each cell holds ~40-60
// points — scanning a cell is a contiguous memory read that the warp
// coalesces cleanly.
//
// Correctness: the CPU path in Association::findCorresp (kdtree radius
// 2.236 m, plane-gate dist² < 5 i.e. dist < sqrt(5) ≈ 2.236 m) silently
// rejects GPU candidates beyond that gate, so the GPU must GUARANTEE it
// finds every point within 2.236 m of the query — else the IEKF sees
// fewer correspondences than the CPU baseline (→ more iterations, drift).
//
// For a query anywhere in the center cell, the minimum distance to the
// outside of a (2*S+1)-cell cube is S * voxel. So the guaranteed search
// radius r_guaranteed = kSearchShellRadius * kVoxelSize must exceed the
// CPU gate 2.236 m. (1.2 m × 2 = 2.4 m ✓)
constexpr float kVoxelSize = 1.2f;
constexpr float kInvVoxelSize = 1.0f / kVoxelSize;
constexpr int   kSearchShellRadius = 2;    // ±2 → 5×5×5 = 125 cells
constexpr int   kSearchShellDiam   = 2 * kSearchShellRadius + 1;

// Hash table size. Aim for ~4 points per bucket on average for a 1M map.
// Power of two enables fast modulo via bit-mask.
constexpr int kNumBucketsLog2 = 18;  // 256K buckets
constexpr int kNumBuckets = 1 << kNumBucketsLog2;
constexpr uint32_t kBucketMask = kNumBuckets - 1;

#define CUDA_CHECK(call)                                                    \
    do {                                                                    \
        cudaError_t err__ = (call);                                         \
        if (err__ != cudaSuccess) {                                         \
            fprintf(stderr, "[CudaMap] CUDA error %s at %s:%d: %s\n",       \
                    #call, __FILE__, __LINE__, cudaGetErrorString(err__));  \
            std::abort();                                                   \
        }                                                                   \
    } while (0)

// Simple multi-prime spatial hash. Three large primes XORed together
// distribute neighboring cells to different buckets so adjacent cells
// don't collide systematically.
__device__ __host__ __forceinline__ uint32_t hash_cell(int cx, int cy, int cz) {
    constexpr uint32_t P1 = 73856093u;
    constexpr uint32_t P2 = 19349663u;
    constexpr uint32_t P3 = 83492791u;
    return (static_cast<uint32_t>(cx) * P1) ^
           (static_cast<uint32_t>(cy) * P2) ^
           (static_cast<uint32_t>(cz) * P3);
}

__device__ __forceinline__ int floor_div(float x, float inv_s) {
    // floor(x * inv_s) with negative-correct rounding
    return static_cast<int>(floorf(x * inv_s));
}

__device__ __forceinline__ void insert_topk(float* arr_dist, int* arr_idx,
                                            int k, float dist, int idx)
{
    if (dist >= arr_dist[k - 1]) return;
    int j = k - 1;
    while (j > 0 && arr_dist[j - 1] > dist) {
        arr_dist[j] = arr_dist[j - 1];
        arr_idx[j] = arr_idx[j - 1];
        --j;
    }
    arr_dist[j] = dist;
    arr_idx[j] = idx;
}

// Phase 1: compute each point's bucket index.
__global__ void compute_buckets_kernel(const float4* __restrict__ points,
                                       int n_points,
                                       uint32_t* __restrict__ out_buckets)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_points) return;
    const float4 p = points[i];
    const int cx = floor_div(p.x, kInvVoxelSize);
    const int cy = floor_div(p.y, kInvVoxelSize);
    const int cz = floor_div(p.z, kInvVoxelSize);
    out_buckets[i] = hash_cell(cx, cy, cz) & kBucketMask;
}

// Phase 2: gather points by sorted index (after CUB sorts indices by bucket).
__global__ void gather_points_kernel(const float4* __restrict__ src_points,
                                     const int* __restrict__ sorted_indices,
                                     int n_points,
                                     float4* __restrict__ dst_points)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_points) return;
    dst_points[i] = src_points[sorted_indices[i]];
}

// Initialize bucket_starts to n_points (sentinel = "no points >= this bucket").
// Backward-scan with min(bucket_starts[b], bucket_starts[b+1]) will produce
// the correct monotone bucket_starts where:
//   bucket_starts[b] = first sorted index whose bucket >= b
// Empty buckets b will have bucket_starts[b] == bucket_starts[b+1] (zero range).
__global__ void init_bucket_starts_kernel(int* __restrict__ bucket_starts,
                                          int n_points)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i > kNumBuckets) return;
    bucket_starts[i] = n_points;
}

// Phase 3a: scatter the min sorted index into each populated bucket using
// atomicMin. This gives the correct value for buckets with points; empty
// buckets retain n_points. O(n_points) work, no per-thread inner loop.
__global__ void scatter_bucket_starts_kernel(const uint32_t* __restrict__ sorted_buckets,
                                             int n_points,
                                             int* __restrict__ bucket_starts)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_points) return;
    const uint32_t b = sorted_buckets[i];
    atomicMin(&bucket_starts[b], i);
}

// Phase 3b: backward suffix-min scan. Sequential on a single GPU thread
// to avoid the read/write races of a doubling-stride parallel scan.
// 256K iterations × ~1ns/iter ≈ 0.3ms — well within budget.
__global__ void backward_suffix_min_kernel(int* __restrict__ bucket_starts,
                                           int n_buckets)
{
    if (threadIdx.x != 0 || blockIdx.x != 0) return;
    for (int b = n_buckets - 1; b >= 0; --b) {
        if (bucket_starts[b + 1] < bucket_starts[b]) {
            bucket_starts[b] = bucket_starts[b + 1];
        }
    }
}

// insert_topk variant that also tracks the neighbor's float4 coords.
__device__ __forceinline__ void insert_topk_pt(float* arr_dist, float4* arr_pt,
                                                int k, float dist, float4 pt)
{
    if (dist >= arr_dist[k - 1]) return;
    int j = k - 1;
    while (j > 0 && arr_dist[j - 1] > dist) {
        arr_dist[j] = arr_dist[j - 1];
        arr_pt[j] = arr_pt[j - 1];
        --j;
    }
    arr_dist[j] = dist;
    arr_pt[j] = pt;
}

// Search kernel: per-query, scan surrounding cells in shell-order until
// the k-th best distance is guaranteed-covered (see kSearchShellRadius),
// finding top-k. Outputs neighbor coordinates (float4) and squared
// distances directly, avoiding the need for a host-side index → point
// lookup table. One block per kThreadsPerBlock queries, one thread per
// query.
__global__ void search_kernel(const float4* __restrict__ sorted_points,
                              const int* __restrict__ bucket_starts,
                              int n_points,
                              const float4* __restrict__ queries,
                              int n_queries,
                              float4* __restrict__ out_neighbors,
                              float* __restrict__ out_dist_sq,
                              int k)
{
    const int qid = blockIdx.x * blockDim.x + threadIdx.x;
    if (qid >= n_queries) return;

    const float4 q = queries[qid];
    // Inactive queries (points outside the active 4-knot window) are
    // marked with NaN coords by Association::findCorresp. Skip them
    // cheaply instead of relying on NaN falling through insert_topk's
    // comparison guard, which then corrupts slot 0 with a NaN distance.
    if (!isfinite(q.x) || !isfinite(q.y) || !isfinite(q.z)) {
        for (int i = 0; i < k; ++i) {
            out_dist_sq[qid * k + i] = INFINITY;
            out_neighbors[qid * k + i] = make_float4(0.f, 0.f, 0.f, 0.f);
        }
        return;
    }
    const int qcx = floor_div(q.x, kInvVoxelSize);
    const int qcy = floor_div(q.y, kInvVoxelSize);
    const int qcz = floor_div(q.z, kInvVoxelSize);

    // Per-thread top-k in registers (distance + point coords).
    float local_dist[kMaxK];
    float4 local_pt[kMaxK];
#pragma unroll
    for (int i = 0; i < kMaxK; ++i) {
        local_dist[i] = INFINITY;
        local_pt[i] = make_float4(0.f, 0.f, 0.f, 0.f);
    }

    // Shell-ordered cell scan with early termination.
    //
    // After scanning all cells at offset ≤ shell, we have examined every
    // point within radius (shell * voxel) of the query. If the current
    // k-th best distance is ≤ (shell * voxel)², no future shell can
    // produce a closer candidate (points in cells at offset ≥ shell+1
    // are at least shell*voxel away from the query), so we're done.
    //
    // This is the canonical radius-search kNN prune. In typical LiDAR
    // densities the first shell satisfies the k=5 query, so most calls
    // touch only 1-27 cells rather than all (2*S+1)^3.
    for (int shell = 0; shell <= kSearchShellRadius; ++shell) {
        for (int dz = -shell; dz <= shell; ++dz)
        for (int dy = -shell; dy <= shell; ++dy)
        for (int dx = -shell; dx <= shell; ++dx) {
            // Skip cells belonging to inner shells (already scanned).
            const int m = max(max(abs(dx), abs(dy)), abs(dz));
            if (m != shell) continue;

            const int cx = qcx + dx;
            const int cy = qcy + dy;
            const int cz = qcz + dz;
            const uint32_t bucket = hash_cell(cx, cy, cz) & kBucketMask;

            const int start = bucket_starts[bucket];
            const int end   = bucket_starts[bucket + 1];

            for (int p = start; p < end; ++p) {
                const float4 pt = sorted_points[p];
                // Hash-collision filter: a different cell may hash to
                // the same bucket; check that this point actually
                // belongs to the cell we're examining.
                const int pcx = floor_div(pt.x, kInvVoxelSize);
                const int pcy = floor_div(pt.y, kInvVoxelSize);
                const int pcz = floor_div(pt.z, kInvVoxelSize);
                if (pcx != cx || pcy != cy || pcz != cz) continue;

                const float ddx = pt.x - q.x;
                const float ddy = pt.y - q.y;
                const float ddz = pt.z - q.z;
                const float d = ddx*ddx + ddy*ddy + ddz*ddz;
                insert_topk_pt(local_dist, local_pt, k, d, pt);
            }
        }

        const float r_safe = shell * kVoxelSize;
        if (local_dist[k - 1] <= r_safe * r_safe) break;
    }

    // Write top-k.
    for (int i = 0; i < k; ++i) {
        out_dist_sq[qid * k + i] = local_dist[i];
        out_neighbors[qid * k + i] = local_pt[i];
    }
}

}  // namespace

struct CudaMap::Impl {
    // GPU-resident sorted map (after hash-grid build).
    float4* d_sorted_points = nullptr;
    int n_points = 0;
    int points_capacity = 0;

    // Hash-grid metadata.
    int* d_bucket_starts = nullptr;  // kNumBuckets + 1 entries

    // Build scratch (kept around so we don't reallocate every update).
    float4* d_input_points = nullptr;
    uint32_t* d_buckets = nullptr;
    uint32_t* d_sorted_buckets = nullptr;
    int* d_input_indices = nullptr;
    int* d_sorted_indices = nullptr;
    int build_capacity = 0;

    void* d_cub_temp = nullptr;
    size_t cub_temp_bytes = 0;

    // Search scratch.
    float4* d_queries = nullptr;
    int queries_capacity = 0;

    float4* d_out_neighbors = nullptr;
    float* d_out_dist_sq = nullptr;
    int out_capacity = 0;

    void ensure_build_capacity(int n) {
        if (n <= build_capacity) return;
        if (d_input_points)  CUDA_CHECK(cudaFree(d_input_points));
        if (d_sorted_points) CUDA_CHECK(cudaFree(d_sorted_points));
        if (d_buckets)        CUDA_CHECK(cudaFree(d_buckets));
        if (d_sorted_buckets) CUDA_CHECK(cudaFree(d_sorted_buckets));
        if (d_input_indices)  CUDA_CHECK(cudaFree(d_input_indices));
        if (d_sorted_indices) CUDA_CHECK(cudaFree(d_sorted_indices));
        build_capacity = n + (n >> 2);
        points_capacity = build_capacity;
        CUDA_CHECK(cudaMalloc(&d_input_points,   build_capacity * sizeof(float4)));
        CUDA_CHECK(cudaMalloc(&d_sorted_points,  build_capacity * sizeof(float4)));
        CUDA_CHECK(cudaMalloc(&d_buckets,        build_capacity * sizeof(uint32_t)));
        CUDA_CHECK(cudaMalloc(&d_sorted_buckets, build_capacity * sizeof(uint32_t)));
        CUDA_CHECK(cudaMalloc(&d_input_indices,  build_capacity * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&d_sorted_indices, build_capacity * sizeof(int)));
    }

    void ensure_bucket_storage() {
        if (d_bucket_starts) return;
        CUDA_CHECK(cudaMalloc(&d_bucket_starts, (kNumBuckets + 1) * sizeof(int)));
    }

    void ensure_queries_capacity(int n) {
        if (n <= queries_capacity) return;
        if (d_queries) CUDA_CHECK(cudaFree(d_queries));
        queries_capacity = n + (n >> 2);
        CUDA_CHECK(cudaMalloc(&d_queries, queries_capacity * sizeof(float4)));
    }

    void ensure_out_capacity(int n) {
        if (n <= out_capacity) return;
        if (d_out_neighbors) CUDA_CHECK(cudaFree(d_out_neighbors));
        if (d_out_dist_sq) CUDA_CHECK(cudaFree(d_out_dist_sq));
        out_capacity = n + (n >> 2);
        CUDA_CHECK(cudaMalloc(&d_out_neighbors, out_capacity * sizeof(float4)));
        CUDA_CHECK(cudaMalloc(&d_out_dist_sq, out_capacity * sizeof(float)));
    }

    void ensure_cub_temp(size_t bytes) {
        if (bytes <= cub_temp_bytes) return;
        if (d_cub_temp) CUDA_CHECK(cudaFree(d_cub_temp));
        cub_temp_bytes = bytes + (bytes >> 2);
        CUDA_CHECK(cudaMalloc(&d_cub_temp, cub_temp_bytes));
    }

    ~Impl() {
        if (d_input_points)   cudaFree(d_input_points);
        if (d_sorted_points)  cudaFree(d_sorted_points);
        if (d_buckets)        cudaFree(d_buckets);
        if (d_sorted_buckets) cudaFree(d_sorted_buckets);
        if (d_input_indices)  cudaFree(d_input_indices);
        if (d_sorted_indices) cudaFree(d_sorted_indices);
        if (d_bucket_starts)  cudaFree(d_bucket_starts);
        if (d_cub_temp)       cudaFree(d_cub_temp);
        if (d_queries)        cudaFree(d_queries);
        if (d_out_neighbors)  cudaFree(d_out_neighbors);
        if (d_out_dist_sq)    cudaFree(d_out_dist_sq);
    }
};

CudaMap::CudaMap() : impl_(new Impl()) {}
CudaMap::~CudaMap() { delete impl_; }

void CudaMap::update(const pcl::PointXYZINormal* points, std::size_t n_in) {
    RESPLE_NVTX("CudaMap::update");
    const int n = static_cast<int>(n_in);
    impl_->n_points = n;
    if (n == 0) {
        return;
    }

    impl_->ensure_build_capacity(n);
    impl_->ensure_bucket_storage();

    // Stage points → float4 on host, then upload to d_input_points.
    {
        RESPLE_PROF_PHASE_CPU("update.stage_points_host");
        std::vector<float4> staged(n);
        for (int i = 0; i < n; ++i) {
            const auto& p = points[i];
            staged[i] = {p.x, p.y, p.z, p.intensity};
        }
        RESPLE_PROF_PHASE_GPU("update.h2d_points");
        CUDA_CHECK(cudaMemcpy(impl_->d_input_points, staged.data(),
                              n * sizeof(float4), cudaMemcpyHostToDevice));
    }

    // Phase 1: compute each point's bucket.
    {
        RESPLE_PROF_PHASE_GPU("update.compute_buckets_kernel");
        const int blocks = (n + kThreadsPerBlock - 1) / kThreadsPerBlock;
        compute_buckets_kernel<<<blocks, kThreadsPerBlock>>>(
            impl_->d_input_points, n, impl_->d_buckets);
    }

    // Initialize indices [0, 1, ..., n-1] on the device. CUB needs both
    // arrays present, and we'll sort the indices to match the sorted
    // bucket order.
    {
        RESPLE_PROF_PHASE_CPU("update.stage_indices_host");
        // Could use cub::DeviceFor or a tiny kernel; a small kernel is
        // simpler (and we don't pay for a fresh cub include).
        std::vector<int> h_idx(n);
        for (int i = 0; i < n; ++i) h_idx[i] = i;
        RESPLE_PROF_PHASE_GPU("update.h2d_indices");
        CUDA_CHECK(cudaMemcpy(impl_->d_input_indices, h_idx.data(),
                              n * sizeof(int), cudaMemcpyHostToDevice));
    }

    // Phase 2: sort (bucket, index) pairs by bucket.
    {
        RESPLE_PROF_PHASE_GPU("update.sort_pairs");
        size_t needed = 0;
        cub::DeviceRadixSort::SortPairs(
            nullptr, needed,
            impl_->d_buckets, impl_->d_sorted_buckets,
            impl_->d_input_indices, impl_->d_sorted_indices,
            n);
        impl_->ensure_cub_temp(needed);
        cub::DeviceRadixSort::SortPairs(
            impl_->d_cub_temp, impl_->cub_temp_bytes,
            impl_->d_buckets, impl_->d_sorted_buckets,
            impl_->d_input_indices, impl_->d_sorted_indices,
            n);
    }

    // Phase 3: gather points by sorted index → d_sorted_points.
    {
        RESPLE_PROF_PHASE_GPU("update.gather_points_kernel");
        const int blocks = (n + kThreadsPerBlock - 1) / kThreadsPerBlock;
        gather_points_kernel<<<blocks, kThreadsPerBlock>>>(
            impl_->d_input_points, impl_->d_sorted_indices, n,
            impl_->d_sorted_points);
    }

    // Phase 4: build per-bucket start offsets via atomic-scatter +
    // parallel backward min-scan. After this:
    //   bucket_starts[b] = first sorted index whose bucket >= b
    // and bucket_starts is monotone non-decreasing in b.
    {
        RESPLE_PROF_PHASE_GPU("update.bucket_starts");
        const int init_blocks = (kNumBuckets + 1 + kThreadsPerBlock - 1) / kThreadsPerBlock;
        init_bucket_starts_kernel<<<init_blocks, kThreadsPerBlock>>>(
            impl_->d_bucket_starts, n);

        const int scatter_blocks = (n + kThreadsPerBlock - 1) / kThreadsPerBlock;
        scatter_bucket_starts_kernel<<<scatter_blocks, kThreadsPerBlock>>>(
            impl_->d_sorted_buckets, n, impl_->d_bucket_starts);

        // Backward suffix-min scan (sequential single-thread; ~0.3ms).
        backward_suffix_min_kernel<<<1, 1>>>(
            impl_->d_bucket_starts, kNumBuckets);
    }

    // No DtoH copy / host mirror — search kernel returns neighbor coords
    // directly, so the host doesn't need a copy of the sorted map.
}

void CudaMap::batch_search(const pcl::PointXYZINormal* queries, std::size_t n_queries_in,
                           int k,
                           std::vector<pcl::PointXYZINormal>& neighbors,
                           std::vector<float>& dist_sq)
{
    RESPLE_NVTX("CudaMap::batch_search");
    const int n_queries = static_cast<int>(n_queries_in);
    {
        RESPLE_PROF_PHASE_CPU("search.alloc_outputs");
        neighbors.assign(n_queries * k, pcl::PointXYZINormal{});
        dist_sq.assign(n_queries * k, INFINITY);
    }

    if (n_queries == 0 || impl_->n_points == 0) {
        prof::frame_done();
        return;
    }
    if (k <= 0 || k > kMaxK) {
        throw std::runtime_error(
            "CudaMap::batch_search: k out of range [1, kMaxK]");
    }

    impl_->ensure_queries_capacity(n_queries);
    impl_->ensure_out_capacity(n_queries * k);

    {
        RESPLE_PROF_PHASE_CPU("search.stage_queries_host");
        std::vector<float4> staged_q(n_queries);
        for (int i = 0; i < n_queries; ++i) {
            const auto& p = queries[i];
            staged_q[i] = {p.x, p.y, p.z, 0.0f};
        }
        RESPLE_PROF_PHASE_GPU("search.h2d_queries");
        CUDA_CHECK(cudaMemcpy(impl_->d_queries, staged_q.data(),
                              n_queries * sizeof(float4),
                              cudaMemcpyHostToDevice));
    }

    {
        RESPLE_PROF_PHASE_GPU("search.search_kernel");
        const int blocks = (n_queries + kThreadsPerBlock - 1) / kThreadsPerBlock;
        search_kernel<<<blocks, kThreadsPerBlock>>>(
            impl_->d_sorted_points, impl_->d_bucket_starts, impl_->n_points,
            impl_->d_queries, n_queries,
            impl_->d_out_neighbors, impl_->d_out_dist_sq, k);
        CUDA_CHECK(cudaGetLastError());
    }

    // Stage neighbor coords back as float4, then unpack into PCL points
    // on the CPU. (Could DtoH directly into a PCL buffer if we cast,
    // but PCL's struct layout has padding so a separate stage is safer.)
    std::vector<float4> staged_n(n_queries * k);
    {
        RESPLE_PROF_PHASE_GPU("search.d2h_neighbors");
        CUDA_CHECK(cudaMemcpy(staged_n.data(), impl_->d_out_neighbors,
                              n_queries * k * sizeof(float4),
                              cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(dist_sq.data(), impl_->d_out_dist_sq,
                              n_queries * k * sizeof(float),
                              cudaMemcpyDeviceToHost));
    }

    {
        RESPLE_PROF_PHASE_CPU("search.unpack_neighbors_host");
        for (int i = 0; i < n_queries * k; ++i) {
            auto& dst = neighbors[i];
            dst.x = staged_n[i].x;
            dst.y = staged_n[i].y;
            dst.z = staged_n[i].z;
            dst.intensity = staged_n[i].w;
            dst.normal_x = 0;
            dst.normal_y = 0;
            dst.normal_z = 0;
            dst.curvature = 0;
        }
    }

    // batch_search is the natural per-frame cadence (one call per IEKF
    // iteration; iterations are bounded by max_iter — the periodic dump
    // will catch any phase divergence even with multiple iter/frame).
    prof::frame_done();
}

std::size_t CudaMap::n_points() const {
    return impl_ ? static_cast<std::size_t>(impl_->n_points) : 0;
}

bool CudaMap::empty() const { return n_points() == 0; }

}  // namespace resple_gpu
