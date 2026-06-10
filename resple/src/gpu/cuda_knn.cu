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

// On a CUDA error, throw rather than std::abort(). update()/batch_search()
// catch the throw, latch Impl::failed, and report the map empty so RESPLE
// falls back to the ikd-tree CPU path instead of taking the whole node down.
// (CUDA errors are usually sticky — the context is unusable — so latching and
// never touching the GPU again is exactly the right degradation.) This macro
// is only used in host code; never inside a __global__ kernel.
#define CUDA_CHECK(call)                                                    \
    do {                                                                    \
        cudaError_t err__ = (call);                                         \
        if (err__ != cudaSuccess) {                                         \
            char msg__[512];                                                \
            snprintf(msg__, sizeof(msg__),                                  \
                     "[CudaMap] CUDA error %s at %s:%d: %s",                \
                     #call, __FILE__, __LINE__, cudaGetErrorString(err__)); \
            throw std::runtime_error(msg__);                                \
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

// Phase 1: compute each point's bucket index AND its cell ID. The cell ID
// is stashed alongside the point so the search kernel can do an integer
// equality check for hash-collision verification instead of recomputing
// three floorf() per candidate — the dominant inner-loop cost.
__global__ void compute_buckets_kernel(const float4* __restrict__ points,
                                       int n_points,
                                       uint32_t* __restrict__ out_buckets,
                                       int3* __restrict__ out_cell_ids)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_points) return;
    const float4 p = points[i];
    const int cx = floor_div(p.x, kInvVoxelSize);
    const int cy = floor_div(p.y, kInvVoxelSize);
    const int cz = floor_div(p.z, kInvVoxelSize);
    out_buckets[i]  = hash_cell(cx, cy, cz) & kBucketMask;
    out_cell_ids[i] = make_int3(cx, cy, cz);
}

// Phase 2: gather points AND their precomputed cell IDs by sorted index
// (one pass — two gathers share the index load).
__global__ void gather_points_kernel(const float4* __restrict__ src_points,
                                     const int3*   __restrict__ src_cell_ids,
                                     const int*    __restrict__ sorted_indices,
                                     int n_points,
                                     float4* __restrict__ dst_points,
                                     int3*   __restrict__ dst_cell_ids)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_points) return;
    const int idx = sorted_indices[i];
    dst_points[i]   = src_points[idx];
    dst_cell_ids[i] = src_cell_ids[idx];
}

// Fill d_out[i] = i for i in [0, n). Replaces the host-side std::vector<int>
// of 0..n-1 that we used to copy to the device each frame — saves one
// n*4-byte alloc + H2D copy on a 1M-point map (~4 MB/frame on Nano).
__global__ void iota_kernel(int* __restrict__ d_out, int n)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    d_out[i] = i;
}

// Set bucket_starts[kNumBuckets] = n. Needed as the exclusive-scan output
// only populates [0..kNumBuckets); the search kernel reads
// bucket_starts[bucket + 1] so the final slot must be the global count.
__global__ void set_bucket_sentinel_kernel(int* __restrict__ bucket_starts,
                                           int n_points)
{
    if (threadIdx.x != 0 || blockIdx.x != 0) return;
    bucket_starts[kNumBuckets] = n_points;
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
                              const int3*   __restrict__ sorted_cell_ids,
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
                // Hash-collision filter: a different cell may hash to
                // the same bucket. Compare against the precomputed
                // per-point cell ID (3×int32 equality) rather than
                // recomputing via three floorf() calls per candidate —
                // this inner loop is the hottest point in the kernel.
                const int3 cid = sorted_cell_ids[p];
                if (cid.x != cx || cid.y != cy || cid.z != cz) continue;

                const float4 pt = sorted_points[p];
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

// Pinned/zero-copy host buffer. On integrated GPUs (Tegra/Jetson) the
// device pointer aliases host memory — kernel accesses bypass the
// pageable copy entirely. On discrete GPUs the host side stays pinned
// for fast async H2D but the device pointer is a separate VRAM alloc.
struct PinnedBuf {
    void*  host = nullptr;
    void*  dev  = nullptr;
    size_t bytes = 0;
    bool   mapped = false;   // host and dev alias the same DRAM

    void free_all() {
        // Silent free: called from both ensure() (reallocation) and ~Impl()
        // (destruction). Matching the surrounding destructor style — on
        // shutdown we'd rather let the program exit cleanly than abort.
        if (host) cudaFreeHost(host);
        if (!mapped && dev) cudaFree(dev);
        host = nullptr; dev = nullptr; bytes = 0;
    }
    void ensure(size_t needed, bool integrated) {
        if (needed <= bytes && mapped == integrated) return;
        free_all();
        bytes = needed + (needed >> 2);
        mapped = integrated;
        if (mapped) {
            CUDA_CHECK(cudaHostAlloc(&host, bytes, cudaHostAllocMapped));
            CUDA_CHECK(cudaHostGetDevicePointer(&dev, host, 0));
        } else {
            CUDA_CHECK(cudaHostAlloc(&host, bytes, cudaHostAllocDefault));
            CUDA_CHECK(cudaMalloc(&dev, bytes));
        }
    }
};

struct CudaMap::Impl {
    // True on Tegra/Jetson. On integrated SoCs GPU and CPU share physical
    // DRAM, so mapped host pointers are the fastest path; explicit
    // cudaMemcpy is pure overhead. Detected lazily on first use — we
    // can't probe the device from a static initializer without forcing
    // CUDA context creation before the user has had a chance to
    // cudaSetDevice.
    bool integrated = false;
    bool integrated_detected = false;
    // Sticky GPU-failure latch. Set by update()/batch_search() if any CUDA op
    // throws. Once set, n_points() reports 0 so CudaMap::empty() is true and
    // RESPLE falls back to the ikd-tree CPU path. Not cleared by clear() — a
    // real CUDA fault leaves the context unusable for the rest of the process.
    bool failed = false;
    cudaStream_t stream = 0;

    // GPU-resident sorted map (after hash-grid build).
    float4* d_sorted_points = nullptr;
    int3*   d_sorted_cell_ids = nullptr;   // (cx, cy, cz) per sorted point
    int n_points = 0;
    int points_capacity = 0;

    // Hash-grid metadata.
    int* d_bucket_starts = nullptr;  // kNumBuckets + 1 entries
    int* d_bucket_counts = nullptr;  // histogram scratch, kNumBuckets entries

    // Build scratch (kept around so we don't reallocate every update).
    PinnedBuf staged_points;   // pinned host float4[n]; .dev = d_input_points on discrete
    uint32_t* d_buckets = nullptr;
    uint32_t* d_sorted_buckets = nullptr;
    int3*     d_cell_ids = nullptr;         // unsorted: output of compute_buckets_kernel
    int* d_input_indices = nullptr;
    int* d_sorted_indices = nullptr;
    int build_capacity = 0;

    // CUB temp caches. Queried once per distinct operation & n; cached.
    void* d_cub_sort = nullptr;       size_t cub_sort_bytes = 0;
    void* d_cub_hist = nullptr;       size_t cub_hist_bytes = 0;
    void* d_cub_scan = nullptr;       size_t cub_scan_bytes = 0;
    int cub_query_n = 0;   // last n for which sort/hist sizes were queried

    // Search staging.
    PinnedBuf staged_queries;  // pinned host float4[n_queries]; .dev = d_queries on discrete
    int queries_capacity = 0;

    PinnedBuf staged_out_nbr;  // pinned host float4[n_queries * k]; .dev = d_out_neighbors on discrete
    PinnedBuf staged_out_dst;  // pinned host float  [n_queries * k]; .dev = d_out_dist_sq  on discrete
    int out_capacity = 0;

    void maybe_detect_integrated() {
        if (integrated_detected) return;
        int dev_id = 0;
        CUDA_CHECK(cudaGetDevice(&dev_id));
        int is_integrated = 0;
        CUDA_CHECK(cudaDeviceGetAttribute(&is_integrated,
                                          cudaDevAttrIntegrated, dev_id));
        integrated = (is_integrated != 0);
        integrated_detected = true;
    }

    void ensure_build_capacity(int n) {
        staged_points.ensure(n * sizeof(float4), integrated);
        if (n <= build_capacity) return;
        if (d_sorted_points)   CUDA_CHECK(cudaFree(d_sorted_points));
        if (d_sorted_cell_ids) CUDA_CHECK(cudaFree(d_sorted_cell_ids));
        if (d_buckets)        CUDA_CHECK(cudaFree(d_buckets));
        if (d_sorted_buckets) CUDA_CHECK(cudaFree(d_sorted_buckets));
        if (d_cell_ids)       CUDA_CHECK(cudaFree(d_cell_ids));
        if (d_input_indices)  CUDA_CHECK(cudaFree(d_input_indices));
        if (d_sorted_indices) CUDA_CHECK(cudaFree(d_sorted_indices));
        build_capacity = n + (n >> 2);
        points_capacity = build_capacity;
        CUDA_CHECK(cudaMalloc(&d_sorted_points,   build_capacity * sizeof(float4)));
        CUDA_CHECK(cudaMalloc(&d_sorted_cell_ids, build_capacity * sizeof(int3)));
        CUDA_CHECK(cudaMalloc(&d_buckets,        build_capacity * sizeof(uint32_t)));
        CUDA_CHECK(cudaMalloc(&d_sorted_buckets, build_capacity * sizeof(uint32_t)));
        CUDA_CHECK(cudaMalloc(&d_cell_ids,       build_capacity * sizeof(int3)));
        CUDA_CHECK(cudaMalloc(&d_input_indices,  build_capacity * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&d_sorted_indices, build_capacity * sizeof(int)));
        // Force re-query of CUB temp sizes (they depend on n).
        cub_query_n = 0;
    }

    void ensure_bucket_storage() {
        if (!d_bucket_starts) {
            CUDA_CHECK(cudaMalloc(&d_bucket_starts, (kNumBuckets + 1) * sizeof(int)));
        }
        if (!d_bucket_counts) {
            CUDA_CHECK(cudaMalloc(&d_bucket_counts, kNumBuckets * sizeof(int)));
        }
    }

    void ensure_queries_capacity(int n) {
        staged_queries.ensure(n * sizeof(float4), integrated);
        queries_capacity = std::max(queries_capacity,
                                    static_cast<int>(staged_queries.bytes / sizeof(float4)));
    }

    void ensure_out_capacity(int n) {
        staged_out_nbr.ensure(n * sizeof(float4), integrated);
        staged_out_dst.ensure(n * sizeof(float),  integrated);
        out_capacity = std::max(out_capacity,
                                static_cast<int>(staged_out_nbr.bytes / sizeof(float4)));
    }

    void ensure_cub_temp(void*& ptr, size_t& cur_bytes, size_t needed) {
        if (needed <= cur_bytes) return;
        if (ptr) CUDA_CHECK(cudaFree(ptr));
        cur_bytes = needed + (needed >> 2);
        CUDA_CHECK(cudaMalloc(&ptr, cur_bytes));
    }

    ~Impl() {
        staged_points.free_all();
        staged_queries.free_all();
        staged_out_nbr.free_all();
        staged_out_dst.free_all();
        if (d_sorted_points)   cudaFree(d_sorted_points);
        if (d_sorted_cell_ids) cudaFree(d_sorted_cell_ids);
        if (d_buckets)        cudaFree(d_buckets);
        if (d_sorted_buckets) cudaFree(d_sorted_buckets);
        if (d_cell_ids)       cudaFree(d_cell_ids);
        if (d_input_indices)  cudaFree(d_input_indices);
        if (d_sorted_indices) cudaFree(d_sorted_indices);
        if (d_bucket_starts)  cudaFree(d_bucket_starts);
        if (d_bucket_counts)  cudaFree(d_bucket_counts);
        if (d_cub_sort)       cudaFree(d_cub_sort);
        if (d_cub_hist)       cudaFree(d_cub_hist);
        if (d_cub_scan)       cudaFree(d_cub_scan);
        if (stream)           cudaStreamDestroy(stream);
    }
};

CudaMap::CudaMap() : impl_(new Impl()) {}
CudaMap::~CudaMap() { delete impl_; }

void CudaMap::update(const pcl::PointXYZINormal* points, std::size_t n_in) {
    RESPLE_NVTX("CudaMap::update");
    // Already in CPU-fallback mode after a prior CUDA failure: stay there.
    if (impl_->failed) { impl_->n_points = 0; return; }
    const int n = static_cast<int>(n_in);
    impl_->n_points = n;
    if (n == 0) {
        return;
    }

    try {
    impl_->maybe_detect_integrated();
    impl_->ensure_build_capacity(n);
    impl_->ensure_bucket_storage();

    // Stage points → float4 in pinned host memory. On integrated GPUs the
    // device pointer aliases this host buffer (zero-copy — the kernel
    // reads through the same physical DRAM). On discrete GPUs we still
    // need an H2D copy, but starting from pinned memory is substantially
    // faster than pageable.
    float4* const h_input = static_cast<float4*>(impl_->staged_points.host);
    float4* const d_input = static_cast<float4*>(impl_->staged_points.dev);
    {
        RESPLE_PROF_PHASE_CPU("update.stage_points_host");
        for (int i = 0; i < n; ++i) {
            const auto& p = points[i];
            h_input[i] = {p.x, p.y, p.z, p.intensity};
        }
        if (!impl_->integrated) {
            RESPLE_PROF_PHASE_GPU("update.h2d_points");
            CUDA_CHECK(cudaMemcpy(d_input, h_input,
                                  n * sizeof(float4),
                                  cudaMemcpyHostToDevice));
        }
    }

    // Phase 1: compute each point's bucket + cell ID (stored alongside
    // points so the search kernel can skip 3 floorf() per candidate).
    {
        RESPLE_PROF_PHASE_GPU("update.compute_buckets_kernel");
        const int blocks = (n + kThreadsPerBlock - 1) / kThreadsPerBlock;
        compute_buckets_kernel<<<blocks, kThreadsPerBlock>>>(
            d_input, n, impl_->d_buckets, impl_->d_cell_ids);
        CUDA_CHECK(cudaGetLastError());
    }

    // Seed indices [0, 1, ..., n-1] on-device. Previously this was a
    // host-side std::vector allocated + filled + copied; a 1-line
    // kernel does the same work without any host allocation or copy.
    {
        RESPLE_PROF_PHASE_GPU("update.iota_kernel");
        const int blocks = (n + kThreadsPerBlock - 1) / kThreadsPerBlock;
        iota_kernel<<<blocks, kThreadsPerBlock>>>(impl_->d_input_indices, n);
        CUDA_CHECK(cudaGetLastError());
    }

    // Phase 2: sort (bucket, index) pairs by bucket. Cache the CUB temp
    // size across calls (requery only when n changes) — previously we
    // re-queried every frame, which isn't free.
    {
        RESPLE_PROF_PHASE_GPU("update.sort_pairs");
        if (n != impl_->cub_query_n) {
            size_t needed = 0;
            cub::DeviceRadixSort::SortPairs(
                nullptr, needed,
                impl_->d_buckets, impl_->d_sorted_buckets,
                impl_->d_input_indices, impl_->d_sorted_indices,
                n);
            impl_->ensure_cub_temp(impl_->d_cub_sort, impl_->cub_sort_bytes, needed);
        }
        cub::DeviceRadixSort::SortPairs(
            impl_->d_cub_sort, impl_->cub_sort_bytes,
            impl_->d_buckets, impl_->d_sorted_buckets,
            impl_->d_input_indices, impl_->d_sorted_indices,
            n);
    }

    // Phase 3: gather points + cell IDs by sorted index (one kernel,
    // shared index load).
    {
        RESPLE_PROF_PHASE_GPU("update.gather_points_kernel");
        const int blocks = (n + kThreadsPerBlock - 1) / kThreadsPerBlock;
        gather_points_kernel<<<blocks, kThreadsPerBlock>>>(
            d_input, impl_->d_cell_ids, impl_->d_sorted_indices, n,
            impl_->d_sorted_points, impl_->d_sorted_cell_ids);
        CUDA_CHECK(cudaGetLastError());
    }

    // Phase 4: build per-bucket start offsets as
    //   bucket_starts[b] = first sorted index whose bucket >= b
    //
    // sorted_buckets is already sorted ascending, so:
    //   counts[b]        = #points in bucket b  (histogram)
    //   bucket_starts[b] = Σ_{b'<b} counts[b']  (exclusive scan of counts)
    //
    // This replaces a 3-kernel init+scatter+single-thread-suffix-min
    // pipeline whose last phase was a <<<1,1>>> 256K-iteration loop —
    // the dominant cost on sm_53 (~2-5 ms per update on Nano).
    {
        RESPLE_PROF_PHASE_GPU("update.bucket_starts");
        if (n != impl_->cub_query_n) {
            size_t h_needed = 0;
            cub::DeviceHistogram::HistogramEven(
                nullptr, h_needed,
                impl_->d_sorted_buckets, impl_->d_bucket_counts,
                /*num_levels=*/ kNumBuckets + 1,
                /*lower_level=*/ 0u,
                /*upper_level=*/ static_cast<uint32_t>(kNumBuckets),
                n);
            impl_->ensure_cub_temp(impl_->d_cub_hist, impl_->cub_hist_bytes, h_needed);

            size_t s_needed = 0;
            cub::DeviceScan::ExclusiveSum(
                nullptr, s_needed,
                impl_->d_bucket_counts,
                impl_->d_bucket_starts,
                kNumBuckets);
            impl_->ensure_cub_temp(impl_->d_cub_scan, impl_->cub_scan_bytes, s_needed);
        }

        cub::DeviceHistogram::HistogramEven(
            impl_->d_cub_hist, impl_->cub_hist_bytes,
            impl_->d_sorted_buckets, impl_->d_bucket_counts,
            kNumBuckets + 1, 0u,
            static_cast<uint32_t>(kNumBuckets), n);

        cub::DeviceScan::ExclusiveSum(
            impl_->d_cub_scan, impl_->cub_scan_bytes,
            impl_->d_bucket_counts,
            impl_->d_bucket_starts,
            kNumBuckets);

        set_bucket_sentinel_kernel<<<1, 1>>>(impl_->d_bucket_starts, n);
        CUDA_CHECK(cudaGetLastError());
    }

    impl_->cub_query_n = n;
    // No DtoH copy / host mirror — search kernel returns neighbor coords
    // directly, so the host doesn't need a copy of the sorted map.
    } catch (const std::exception& e) {
        fprintf(stderr, "%s\n[CudaMap] update() failed — disabling GPU map, "
                "falling back to CPU kd-tree path.\n", e.what());
        impl_->failed = true;
        impl_->n_points = 0;
    }
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

    // Sticky failure or empty map: the outputs are already all-miss (INFINITY,
    // assigned above), so findCorresp phase 3 marks every correspondence
    // invalid and the IEKF gets zero GPU correspondences this frame. Combined
    // with empty()==true (failed latch), RESPLE then uses the CPU path.
    if (impl_->failed || n_queries == 0 || impl_->n_points == 0) {
        prof::frame_done();
        return;
    }
    if (k <= 0 || k > kMaxK) {
        throw std::runtime_error(
            "CudaMap::batch_search: k out of range [1, kMaxK]");
    }

    try {
    impl_->maybe_detect_integrated();
    impl_->ensure_queries_capacity(n_queries);
    impl_->ensure_out_capacity(n_queries * k);

    float4* const h_queries = static_cast<float4*>(impl_->staged_queries.host);
    float4* const d_queries = static_cast<float4*>(impl_->staged_queries.dev);
    float4* const h_out_nbr = static_cast<float4*>(impl_->staged_out_nbr.host);
    float4* const d_out_nbr = static_cast<float4*>(impl_->staged_out_nbr.dev);
    float*  const h_out_dst = static_cast<float*> (impl_->staged_out_dst.host);
    float*  const d_out_dst = static_cast<float*> (impl_->staged_out_dst.dev);

    {
        RESPLE_PROF_PHASE_CPU("search.stage_queries_host");
        for (int i = 0; i < n_queries; ++i) {
            const auto& p = queries[i];
            h_queries[i] = {p.x, p.y, p.z, 0.0f};
        }
        if (!impl_->integrated) {
            RESPLE_PROF_PHASE_GPU("search.h2d_queries");
            CUDA_CHECK(cudaMemcpy(d_queries, h_queries,
                                  n_queries * sizeof(float4),
                                  cudaMemcpyHostToDevice));
        }
    }

    {
        RESPLE_PROF_PHASE_GPU("search.search_kernel");
        const int blocks = (n_queries + kThreadsPerBlock - 1) / kThreadsPerBlock;
        search_kernel<<<blocks, kThreadsPerBlock>>>(
            impl_->d_sorted_points, impl_->d_sorted_cell_ids,
            impl_->d_bucket_starts, impl_->n_points,
            d_queries, n_queries,
            d_out_nbr, d_out_dst, k);
        CUDA_CHECK(cudaGetLastError());
    }

    // Bring neighbor coords + distances back to host. On integrated we
    // just need a sync point (mapped buffer already contains the kernel
    // output); on discrete we async-copy back via pinned buffers.
    if (impl_->integrated) {
        RESPLE_PROF_PHASE_GPU("search.sync_mapped");
        CUDA_CHECK(cudaDeviceSynchronize());
    } else {
        RESPLE_PROF_PHASE_GPU("search.d2h_neighbors");
        CUDA_CHECK(cudaMemcpy(h_out_nbr, d_out_nbr,
                              n_queries * k * sizeof(float4),
                              cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(h_out_dst, d_out_dst,
                              n_queries * k * sizeof(float),
                              cudaMemcpyDeviceToHost));
    }

    {
        RESPLE_PROF_PHASE_CPU("search.unpack_neighbors_host");
        const int total = n_queries * k;
        for (int i = 0; i < total; ++i) {
            auto& dst = neighbors[i];
            dst.x = h_out_nbr[i].x;
            dst.y = h_out_nbr[i].y;
            dst.z = h_out_nbr[i].z;
            dst.intensity = h_out_nbr[i].w;
            dst.normal_x = 0;
            dst.normal_y = 0;
            dst.normal_z = 0;
            dst.curvature = 0;
            dist_sq[i] = h_out_dst[i];
        }
    }
    } catch (const std::exception& e) {
        fprintf(stderr, "%s\n[CudaMap] batch_search() failed — disabling GPU "
                "map, falling back to CPU kd-tree path.\n", e.what());
        impl_->failed = true;
        impl_->n_points = 0;
        // Outputs stay all-INFINITY (assigned at entry) → zero correspondences.
    }

    // batch_search is the natural per-frame cadence (one call per IEKF
    // iteration; iterations are bounded by max_iter — the periodic dump
    // will catch any phase divergence even with multiple iter/frame).
    prof::frame_done();
}

std::size_t CudaMap::n_points() const {
    if (!impl_ || impl_->failed) return 0;
    return static_cast<std::size_t>(impl_->n_points);
}

bool CudaMap::empty() const { return n_points() == 0; }

void CudaMap::clear() {
    // Drop the GPU-resident map so empty()==true until the next update(),
    // without freeing the device buffers (the next update() reuses them).
    // Called on lifecycle deactivate so a re-activated node never queries the
    // previous run's stale points before the first mapIncremental re-syncs the
    // GPU. Does NOT clear the sticky `failed` latch (see Impl::failed).
    if (impl_) impl_->n_points = 0;
}

}  // namespace resple_gpu
