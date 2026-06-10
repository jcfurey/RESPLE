// Concurrency regression test for the ikd-Tree Push_Down child-write race
// (HARDENING Phase 2.4; also the Phase 5 "findCorresp stress" target).
//
// WHAT IT GUARDS
//   Phase 2.4 closed a data race where multiple concurrent Nearest_Search
//   readers (RESPLE runs findCorresp as an OpenMP parallel-for) descend
//   overlapping paths and call Push_Down on the same parent/child nodes,
//   racing on the KD_TREE_NODE deletion-propagation fields (need_push_down_*,
//   tree_deleted, point_deleted, down_del_num, invalid_point_num, ...).
//
// HOW IT REPRODUCES IT
//   The test runs in strict phases that mirror RESPLE's mtx_map_ discipline
//   (searches concurrent with each other, never concurrent with mutation):
//     (1) MUTATION phase  — a single thread Deletes boxes (arming
//         need_push_down_to_* on a frontier of subtree roots) and Adds points.
//     (2) SEARCH phase     — a barrier releases N reader threads that all hammer
//         Nearest_Search over the just-deleted region, so many readers push the
//         flagged frontier down at once. This is exactly the racing pattern.
//   Readers and the mutator never overlap; the only concurrency during a search
//   phase is reader-vs-reader (Phase 2.4) plus the kd-tree's own background
//   rebuild thread (Phase 1.5 K1, guarded by search_rw_mutex_).
//
// SIGNAL
//   Run normally it asserts functional sanity (no crash, consistent results).
//   The strong signal is under ThreadSanitizer (see scripts/run_unit_tests.sh
//   notes and resple/test/tsan_suppressions.txt):
//     before Phase 2.4 TSan reports ~16 races on the node fields here;
//     after, those are gone. Two pre-existing, out-of-scope reports remain
//     (rebuild-thread vs Update on father_ptr; the working_flag/search_rw
//     inline-Rebuild lock-order-inversion) and are listed in the suppressions
//     file — they are not introduced by Phase 2.4.

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <random>
#include <shared_mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/point_types.h>

#include "ikd-Tree/ikd_Tree.h"

namespace
{
using Point = pcl::PointXYZINormal;
using Tree = KD_TREE<Point>;
using PointVector = Tree::PointVector;

constexpr float kHalfExtent = 60.0f;  // map points live in [-60, 60]^3
constexpr float kHotExtent = 15.0f;   // deletes + queries concentrate here

Point makePoint(std::mt19937& rng, float center, float half)
{
    std::uniform_real_distribution<float> d(center - half, center + half);
    Point p;
    p.x = d(rng);
    p.y = d(rng);
    p.z = d(rng);
    p.intensity = 0.0f;
    return p;
}

PointVector makeCloud(std::mt19937& rng, int n, float center, float half)
{
    PointVector cloud;
    cloud.reserve(n);
    for (int i = 0; i < n; ++i)
        cloud.push_back(makePoint(rng, center, half));
    return cloud;
}

BoxPointType makeBox(std::mt19937& rng, float side)
{
    std::uniform_real_distribution<float> d(-kHotExtent, kHotExtent - side * 0.5f);
    BoxPointType b;
    for (int a = 0; a < 3; ++a)
    {
        const float lo = d(rng);
        b.vertex_min[a] = lo;
        b.vertex_max[a] = lo + side;
    }
    return b;
}

// Minimal reusable two-phase barrier (C++17; std::barrier is C++20). All
// N participants plus the coordinator meet at each gate.
class Barrier
{
public:
    explicit Barrier(int n) : count_(n), waiting_(0), generation_(0) {}
    void arrive_and_wait()
    {
        std::unique_lock<std::mutex> lk(m_);
        const int gen = generation_;
        if (++waiting_ == count_)
        {
            waiting_ = 0;
            ++generation_;
            cv_.notify_all();
        }
        else
        {
            cv_.wait(lk, [&] { return gen != generation_; });
        }
    }

private:
    std::mutex m_;
    std::condition_variable cv_;
    const int count_;
    int waiting_;
    int generation_;
};
}  // namespace

TEST(IkdTreeConcurrency, ParallelSearchPushDownRace)
{
    std::mt19937 seed_rng(12345);

    // Heap-allocate: a KD_TREE embeds MANUAL_Q Rebuild_Logger, an
    // Operation_Logger_Type[Q_LEN] (~80 MB), so it must NOT live on the stack.
    // The package keeps the live instance in static storage (RESPLE.cpp's
    // `static KD_TREE ikdtree`); here a unique_ptr serves the same purpose.
    auto tree_ptr = std::make_unique<Tree>(0.3f, 0.6f, 0.2f);
    Tree& tree = *tree_ptr;
    tree.Build(makeCloud(seed_rng, 40000, 0.0f, kHalfExtent));  // > Multi_Thread_Rebuild_Point_Num
    ASSERT_GT(tree.size(), 0);

    // Kept modest so the test finishes quickly even under ThreadSanitizer's
    // ~30x slowdown. The Push_Down race fires within the first couple of cycles
    // (pre-fix TSan flags it almost immediately), so this is ample to guard it
    // without inviting long runtimes or the historical rebuild-path hazards
    // (see resple/test/tsan_suppressions.txt) to surface as a hang.
    constexpr int kReaders = 10;
    constexpr int kCycles = 8;
    constexpr int kSearchesPerCycle = 1500;

    Barrier gate(kReaders + 1);  // readers + coordinator
    std::atomic<bool> stop{false};
    std::atomic<long> total_searches{0};
    std::atomic<long> total_results{0};
    std::atomic<int> error_flag{0};

    auto reader_fn = [&](int tid) {
        std::mt19937 rng(1000 + tid);
        long local_searches = 0;
        long local_results = 0;
        while (true)
        {
            gate.arrive_and_wait();  // wait for the mutation phase to finish
            if (stop.load(std::memory_order_relaxed))
                break;
            for (int i = 0; i < kSearchesPerCycle; ++i)
            {
                const Point q = makePoint(rng, 0.0f, kHotExtent);
                PointVector found;
                std::vector<float> dist;
                tree.Nearest_Search(q, 5, found, dist, 50.0f);
                // Length agreement + finiteness: a torn read of the node fields
                // would readily violate these.
                if (found.size() != dist.size())
                    error_flag.store(1, std::memory_order_relaxed);
                for (float dd : dist)
                    if (!std::isfinite(dd) || dd < 0.0f)
                        error_flag.store(2, std::memory_order_relaxed);
                local_results += static_cast<long>(found.size());
                ++local_searches;
            }
            gate.arrive_and_wait();  // signal end of this search phase
        }
        total_searches.fetch_add(local_searches, std::memory_order_relaxed);
        total_results.fetch_add(local_results, std::memory_order_relaxed);
    };

    std::vector<std::thread> readers;
    readers.reserve(kReaders);
    for (int i = 0; i < kReaders; ++i)
        readers.emplace_back(reader_fn, i);

    std::mt19937 mut_rng(99);
    for (int cycle = 0; cycle < kCycles; ++cycle)
    {
        // --- MUTATION phase: single-threaded, no readers active. Box deletes
        // arm need_push_down_to_* on a frontier the readers then push down.
        std::vector<BoxPointType> boxes;
        for (int b = 0; b < 10; ++b)
            boxes.push_back(makeBox(mut_rng, 9.0f));
        tree.Delete_Point_Boxes(boxes);
        PointVector add = makeCloud(mut_rng, 300, 0.0f, kHotExtent);
        tree.Add_Points(add, true);

        gate.arrive_and_wait();  // release readers into the SEARCH phase
        gate.arrive_and_wait();  // wait for the SEARCH phase to complete
    }
    stop.store(true, std::memory_order_relaxed);
    gate.arrive_and_wait();  // release readers so they observe stop and exit
    for (auto& t : readers)
        t.join();

    EXPECT_EQ(error_flag.load(), 0) << "reader observed inconsistent search output";
    EXPECT_GT(total_searches.load(), 0);
    EXPECT_GT(total_results.load(), 0);
    EXPECT_GT(tree.size(), 0);
}

// Aggressive rebuild stress (HARDENING Phase 2.5 #1).
//
// WHAT IT GUARDS
//   The background rebuild thread (multi_thread_rebuild) races a concurrent
//   mutator on KD_TREE_NODE fields: the rebuild reads (*Rebuild_Ptr)->father_ptr
//   and walks ancestors with Update() during the subtree swap, while a mutator's
//   own pull-up Update() writes father_ptr / alpha_* on overlapping ancestors
//   with no common lock. Inherited from upstream HKU-MARS ikd-Tree.
//
// HOW IT REPRODUCES IT
//   Production discipline via a shared_mutex standing in for RESPLE's mtx_map_:
//   searches take it shared, the mutator takes it unique — so the ONLY overlap
//   is each of them vs the kd-tree's internal rebuild thread, exactly the
//   production-reachable pairing. The mutator does large bursty box-deletes in
//   a hot region followed by re-adds, driving subtree balance/deletion ratios
//   past the rebuild criterion so multi-thread rebuilds fire continuously.
//
// SIGNAL
//   Functional sanity normally; under ThreadSanitizer this is the regression
//   gate for the Phase 2.5 #1 fix (father_ptr / alpha_* / working_flag made
//   atomic): pre-fix TSan reports races in multi_thread_rebuild vs Update —
//   post-fix it must be clean WITHOUT suppressions.
TEST(IkdTreeConcurrency, RebuildVsMutatorRace)
{
    std::mt19937 seed_rng(777);
    auto tree_ptr = std::make_unique<Tree>(0.3f, 0.6f, 0.2f);
    Tree& tree = *tree_ptr;
    tree.Build(makeCloud(seed_rng, 40000, 0.0f, kHalfExtent));
    ASSERT_GT(tree.size(), 0);

    constexpr int kReaders = 4;
    // HONEST NOTE on detection power (measured during the Phase 2.5 #1 fix
    // session): the pre-fix rebuild-vs-mutator races are deep in scheduler
    // territory. With this configuration TSan reported ~200 races against
    // the pre-fix tree on a heavily loaded host, and 0 against the fixed
    // tree on identical back-to-back runs — but on a lightly loaded
    // many-core machine the pre-fix window may not be hit at all (several
    // 0-report pre-fix runs were also observed). Treat this test as a
    // CANARY, not a proof of absence: any TSan report here is a real
    // regression and fails the CI gate (halt_on_error=1); a quiet run is
    // necessary, not sufficient. Detection chances are best exactly where
    // the CI job runs it (4-vCPU runner: the 4 spin readers + mutator +
    // rebuild thread oversubscribe the machine, stretching the rebuild
    // thread's flatten/swap windows into the mutator's bursts). Keep the
    // spin readers and hot-region bursts — throttled readers or
    // full-extent spread were measured to make the test fast but blind.
    // Native runtime ~9 s; under TSan expect minutes; under ASan it can
    // exceed the 120 s ctest timeout (run the phased test above for the
    // ASan/UAF angle instead).
    constexpr int kMutCycles = 6;

    std::shared_mutex map_mtx;  // stands in for RESPLE's mtx_map_
    std::atomic<bool> stop{false};
    std::atomic<int> error_flag{0};
    std::atomic<long> total_searches{0};

    auto reader_fn = [&](int tid) {
        std::mt19937 rng(5000 + tid);
        long n = 0;
        while (!stop.load(std::memory_order_relaxed))
        {
            {
                std::shared_lock<std::shared_mutex> lk(map_mtx);
                const Point q = makePoint(rng, 0.0f, kHotExtent);
                PointVector found;
                std::vector<float> dist;
                tree.Nearest_Search(q, 5, found, dist, 50.0f);
                if (found.size() != dist.size())
                    error_flag.store(1, std::memory_order_relaxed);
                for (float dd : dist)
                    if (!std::isfinite(dd) || dd < 0.0f)
                        error_flag.store(2, std::memory_order_relaxed);
                ++n;
            }
            // PURE spin — the reader pressure is load-bearing for detection
            // (see the tuning note above).
        }
        total_searches.fetch_add(n, std::memory_order_relaxed);
    };

    std::vector<std::thread> readers;
    for (int i = 0; i < kReaders; ++i)
        readers.emplace_back(reader_fn, i);

    // Mutator: large bursty deletes + re-adds under the unique lock. Each
    // burst deletes a sizeable slice of the hot region (arming deletion
    // ratios well past the rebuild criterion) and re-adds points, so the
    // background rebuild thread keeps picking up big subtrees while the NEXT
    // burst's pull-up Updates run — the racing overlap under test. The
    // brief unlocked sleep lets the rebuild thread make progress mid-burst.
    std::mt19937 mut_rng(4242);
    for (int cycle = 0; cycle < kMutCycles && !error_flag.load(); ++cycle)
    {
        {
            std::unique_lock<std::shared_mutex> lk(map_mtx);
            std::vector<BoxPointType> boxes;
            // HOT-REGION bursts (not full-extent): repeatedly deleting and
            // re-adding inside the same subtrees drives their deletion /
            // balance ratios past the rebuild criterion, so the multi-thread
            // rebuild keeps firing — full-extent spread leaves the tree
            // balanced and the rebuild thread idle (nothing to race).
            for (int b = 0; b < 5; ++b)
                boxes.push_back(makeBox(mut_rng, 16.0f));
            tree.Delete_Point_Boxes(boxes);
            PointVector add = makeCloud(mut_rng, 2500, 0.0f, kHotExtent);
            tree.Add_Points(add, false);
        }
        // Brief gap so the rebuild thread can pick up the queued rebuild —
        // the NEXT burst then lands while it is mid-rebuild.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    stop.store(true, std::memory_order_relaxed);
    for (auto& t : readers)
        t.join();

    EXPECT_EQ(error_flag.load(), 0) << "reader observed inconsistent search output";
    EXPECT_GT(total_searches.load(), 0);
    EXPECT_GT(tree.size(), 0);
}
