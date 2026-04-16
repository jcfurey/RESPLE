# RESPLE Performance Optimization Plan

## Context

RESPLE runs at ~10-20 Hz processing LiDAR point clouds through a B-spline IEKF pipeline. The main bottlenecks are ikd-Tree k-NN search (~35-40%), Jacobian/spline evaluation (~25-30%), and spline interpolation (~15-20%). The system currently uses OpenMP on the main parallel loops but has several low-hanging performance issues: disabled SIMD flags, forced heap allocations in hot paths, O(k^2) vector insertions in every k-NN query, and busy-wait spin-loops in the tree. Hardware: Ryzen 9 3950X (16c/32t, AVX2+FMA) + RTX 3090 (24GB, compute 8.6).

## Phase 1: Low-Risk CPU Optimizations

Changes that won't alter algorithmic behavior. Each is independently testable.

### 1.1 Enable `-march=native` and add `-ffast-math`
**File:** `resple/CMakeLists.txt` (line 6)
- Change `ENABLE_NATIVE_ARCH` default from `OFF` to `ON`
- Add `-ffast-math` to release compile options (safe for this codebase — no NaN/Inf edge cases relied on)
- Unlocks AVX2+FMA for all Eigen matrix/vector ops
- **Impact: 10-20% overall** | **Effort: trivial**

### 1.2 Fix Nearest_Search allocations and O(k^2) insertion
**File:** `resple/include/ikd-Tree/ikd_Tree.cpp` (lines 427-462)
- Lines 431, 453-454: `vector<float>().swap()` and `PointVector().swap()` force-deallocate and reallocate on every single search call (called 500-50k times per frame)
- Lines 455-458: `insert(begin(), ...)` is O(k) per element → O(k^2) total for k results
- **Fix:** Clear vectors instead of swap-deallocate. Reserve capacity. Build results by push_back then reverse (or index from end)
- **Impact: 3-8%** | **Effort: small**

### 1.3 Pre-allocate Estimator matrices
**File:** `resple/include/Estimator.h` (lines 296-301, 349-351)
- `H`, `innv`, `mat_cov_inv` are dynamically allocated every call to `updateLiDAR` (3-5x per frame)
- Move to class members with `conservativeResize` or pre-allocate to max expected size
- **Impact: 2-5%** | **Effort: small**

### 1.4 Fixed-size plane fitting
**File:** `resple/include/utils/common_utils.h` (line 286)
- `esti_plane` uses `colPivHouseholderQr()` on `Eigen::Dynamic` matrices inside an OpenMP parallel loop
- Replace with fixed-size `Eigen::Matrix<T, 5, 3>` and direct 3x3 normal-equation solve (`A^T A x = A^T b`) which Eigen can fully inline and vectorize
- **Impact: 5-10%** | **Effort: small**

### 1.5 Link Eigen to OpenBLAS
**File:** `resple/CMakeLists.txt`
- Add `find_package(BLAS)` and define `EIGEN_USE_BLAS` if found
- Accelerates the Cholesky and matrix products in the Kalman update (24x24 / 30x30 matrices)
- **Impact: 5-10%** on Kalman update step | **Effort: small**

### 1.6 Reduce PointData struct footprint
**File:** `resple/include/utils/common_utils.h` (PointData struct, lines 347-409)
- Each PointData is ~420+ bytes. The `nearest_points` vector (heap-allocated) and duplicated `q_bl`/`t_bl` (shared per sensor) bloat cache lines
- Move `nearest_points` to thread-local scratch storage in Association.h
- Store sensor extrinsics index instead of copying q_bl/t_bl per point
- **Impact: 5-15%** (cache improvement on the two hottest loops) | **Effort: medium**

**Phase 1 estimated cumulative improvement: 25-45%**

## Phase 2: Algorithmic Improvements

### 2.1 Replace ikd-Tree busy-wait with condition variable
**File:** `resple/include/ikd-Tree/ikd_Tree.cpp` (~15 locations with `usleep(1)`)
- Replace `pthread_mutex` + `usleep(1)` spin pattern with `std::shared_mutex` (readers/writers lock) or `std::condition_variable`
- Search threads are readers (concurrent), rebuild thread is writer (exclusive)
- Key locations: lines 265, 313, 438-450, 1114
- **Impact: 5-15%** under rebuild contention | **Effort: medium**

### 2.2 Lock-free search counter
**File:** `resple/include/ikd-Tree/ikd_Tree.cpp` (lines 438-450)
- Replace `search_flag_mutex` + `search_mutex_counter` with `std::atomic<int>` and CAS
- Currently every OpenMP search thread serializes through this mutex on entry AND exit
- **Impact: 3-5%** | **Effort: small**

### 2.3 Batch spline evaluation
**File:** `resple/include/SplineState.h`
- Points sorted by timestamp often share the same knot interval
- Factor out `prepareInterpolation` and knot coefficient computation; reuse across points in the same interval
- Amortizes the blending matrix multiply and quaternion chain setup
- **Impact: 5-10%** on spline evaluation | **Effort: medium-high**

### 2.4 Async map update
**Files:** `resple/src/RESPLE.cpp` (mapIncremental lines 1629-1666, lasermapFovSegment lines 1576-1627)
- Map insertion and FOV pruning can overlap with the next frame's callback processing
- The ikd-tree already supports concurrent search+rebuild via its logger
- **Impact: reduces frame latency** | **Effort: medium**

**Phase 2 estimated additional improvement: 15-30%**

## Phase 3: GPU Acceleration

### 3.0 CUDA build infrastructure
**File:** `resple/CMakeLists.txt`
- Add `option(ENABLE_CUDA ...)`, `enable_language(CUDA)`, feature-gated compilation
- Add a `resple/src/gpu/` directory for CUDA kernels
- Keep CPU fallback path for non-GPU builds
- **Effort: medium**

### 3.1 GPU batch k-NN via FAISS or custom CUDA
- The ikd-Tree k-NN search is the single largest hotspot (35-40%)
- Approach: periodically export the flat map point array to GPU memory, run batch k-NN for all query points in one kernel launch
- FAISS `GpuIndexFlatL2` can search 500 queries against 1M points in ~1ms on RTX 3090 vs ~15-25ms on CPU
- Hybrid: keep ikd-Tree for incremental map updates, use GPU flat search for association queries
- **Impact: 15-25% overall** | **Effort: high**

### 3.2 GPU point cloud transforms
- Batch body-to-world transform of all points (rotation + translation) as a single CUDA kernel
- Only worthwhile at high point counts (>5k post-downsampling)
- **Impact: 2-5%** | **Effort: medium**

### 3.3 GPU plane fitting
- Batch 5x3 least-squares solves as a single cuSOLVER batched call
- Only worthwhile if Phase 1.4 (fixed-size CPU plane fitting) is still a bottleneck
- **Impact: 2-5%** | **Effort: medium**

## Implementation Order

Priority by impact/effort ratio:
1. **1.1** Enable `-march=native` + `-ffast-math` (trivial, do first)
2. **1.2** Fix Nearest_Search allocations (small, big per-query savings)
3. **1.4** Fixed-size plane fitting (small, removes dynamic dispatch)
4. **1.3** Pre-allocate Estimator matrices (small)
5. **1.5** Link to OpenBLAS (small)
6. **2.2** Lock-free search counter (small)
7. **1.6** Reduce PointData struct (medium, cache improvement)
8. **2.1** Replace busy-wait with condvar (medium)
9. **2.3** Batch spline evaluation (medium-high)
10. **2.4** Async map update (medium)
11. **3.0-3.1** CUDA infrastructure + GPU k-NN (high effort, high reward)

## Verification

After each change:
1. **Build:** `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select resple`
2. **Functional test:** Run TudoRun01 dataset end-to-end, compare output trajectory to baseline
3. **Performance test:** Time full bag playback, compare per-frame processing time from RESPLE's diagnostic output (avg computation time reported in the diagnostics updater)
4. **Before starting:** Establish baseline timing with current code on TudoRun01

## Files to Modify

| File | Phases |
|------|--------|
| `resple/CMakeLists.txt` | 1.1, 1.5, 3.0 |
| `resple/include/ikd-Tree/ikd_Tree.cpp` | 1.2, 2.1, 2.2 |
| `resple/include/ikd-Tree/ikd_Tree.h` | 2.1, 2.2 |
| `resple/include/Estimator.h` | 1.3 |
| `resple/include/Association.h` | 1.6 |
| `resple/include/utils/common_utils.h` | 1.4, 1.6 |
| `resple/include/SplineState.h` | 2.3 |
| `resple/src/RESPLE.cpp` | 2.4 |
| `resple/src/gpu/` (new) | 3.0-3.3 |
