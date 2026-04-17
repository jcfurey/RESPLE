#pragma once

// Lightweight per-phase profiler for the CUDA k-NN path.
//
// Why custom (not just nsys):
//   nsys profile gives flame charts but is heavyweight and inconvenient
//   in the bench harness loop. This header gives:
//     - Per-phase CPU + GPU wall-time accumulators (count, mean, max)
//     - Periodic stderr dump every N frames
//     - Optional NVTX ranges so nsys still lights up the same phases
//   when the user wants the deeper view.
//
// Activation:
//   At runtime, set env var RESPLE_CUDA_PROFILE=1 to enable timing +
//   periodic dumps. Default is disabled — RAII helpers become near-zero
//   cost (one branch on a cached atomic).
//
// Cost when enabled:
//   GpuRange does cudaEventRecord/Synchronize/ElapsedTime around each
//   range. The synchronize SERIALIZES the GPU at every range boundary,
//   so absolute numbers are ~true per-phase but overall throughput will
//   be lower than the un-instrumented run. Use the stats to compare
//   relative phase costs, not to declare end-to-end perf wins.

#include <cuda_runtime.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

#if defined(RESPLE_NVTX_ENABLED)
  #include <nvtx3/nvToolsExt.h>
#endif

namespace resple_gpu {
namespace prof {

inline bool enabled() {
    static const bool v = []() {
        const char* s = std::getenv("RESPLE_CUDA_PROFILE");
        return s && s[0] != '\0' && std::strcmp(s, "0") != 0;
    }();
    return v;
}

struct Stat {
    std::uint64_t n = 0;
    double sum_us = 0.0;
    double max_us = 0.0;
    void add(double us) { ++n; sum_us += us; if (us > max_us) max_us = us; }
};

class Registry {
  public:
    static Registry& instance() { static Registry r; return r; }

    void add_cpu(const char* phase, double us) {
        std::lock_guard<std::mutex> lock(m_);
        cpu_[phase].add(us);
    }
    void add_gpu(const char* phase, double us) {
        std::lock_guard<std::mutex> lock(m_);
        gpu_[phase].add(us);
    }

    void frame_done() {
        const int dump_every = 100;
        const int n = ++frame_count_;
        if (n % dump_every == 0) dump(n);
    }

    void dump(int frame_n) {
        std::lock_guard<std::mutex> lock(m_);
        auto print_section = [](const char* tag,
                                std::unordered_map<std::string, Stat>& m) {
            std::vector<std::pair<std::string, Stat>> rows(m.begin(), m.end());
            std::sort(rows.begin(), rows.end(),
                      [](const auto& a, const auto& b) {
                          return a.second.sum_us > b.second.sum_us;
                      });
            for (const auto& [name, s] : rows) {
                const double avg = s.n ? s.sum_us / s.n : 0.0;
                std::fprintf(stderr,
                    "  %s %-32s n=%-6lu avg=%8.1fus max=%8.1fus total=%9.2fms\n",
                    tag, name.c_str(),
                    static_cast<unsigned long>(s.n), avg, s.max_us,
                    s.sum_us / 1000.0);
            }
        };
        std::fprintf(stderr, "[CudaProfiler] after %d frames\n", frame_n);
        print_section("CPU", cpu_);
        print_section("GPU", gpu_);
        std::fflush(stderr);
    }

  private:
    std::mutex m_;
    std::atomic<int> frame_count_{0};
    std::unordered_map<std::string, Stat> cpu_;
    std::unordered_map<std::string, Stat> gpu_;
};

inline void note_cpu(const char* phase, double us) {
    Registry::instance().add_cpu(phase, us);
}
inline void note_gpu(const char* phase, double us) {
    Registry::instance().add_gpu(phase, us);
}
inline void frame_done() {
    if (enabled()) Registry::instance().frame_done();
}

struct CpuRange {
    explicit CpuRange(const char* phase)
        : phase_(phase), t0_(std::chrono::steady_clock::now()) {}
    ~CpuRange() {
        if (!enabled()) return;
        const auto us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - t0_).count();
        note_cpu(phase_, us);
    }
    const char* phase_;
    std::chrono::steady_clock::time_point t0_;
};

struct GpuRange {
    explicit GpuRange(const char* phase) : phase_(phase) {
        if (!enabled()) return;
        cudaEventCreate(&s_);
        cudaEventCreate(&e_);
        cudaEventRecord(s_);
    }
    ~GpuRange() {
        if (!enabled()) return;
        cudaEventRecord(e_);
        cudaEventSynchronize(e_);
        float ms = 0.f;
        cudaEventElapsedTime(&ms, s_, e_);
        note_gpu(phase_, ms * 1000.0);
        cudaEventDestroy(s_);
        cudaEventDestroy(e_);
    }
    const char* phase_;
    cudaEvent_t s_{}, e_{};
};

#if defined(RESPLE_NVTX_ENABLED)
struct NvtxRange {
    explicit NvtxRange(const char* name) { nvtxRangePushA(name); }
    ~NvtxRange() { nvtxRangePop(); }
};
#endif

}  // namespace prof
}  // namespace resple_gpu

#define RESPLE_CONCAT_(a, b) a##b
#define RESPLE_CONCAT(a, b) RESPLE_CONCAT_(a, b)

#define RESPLE_PROF_CPU(name) \
    ::resple_gpu::prof::CpuRange RESPLE_CONCAT(_cpur_, __LINE__)(name)
#define RESPLE_PROF_GPU(name) \
    ::resple_gpu::prof::GpuRange RESPLE_CONCAT(_gpur_, __LINE__)(name)

#if defined(RESPLE_NVTX_ENABLED)
  #define RESPLE_NVTX(name) \
      ::resple_gpu::prof::NvtxRange RESPLE_CONCAT(_nvtx_, __LINE__)(name)
#else
  #define RESPLE_NVTX(name) do {} while (0)
#endif

// Convenience: emit NVTX (free, lights up nsys) + GPU range (gated on env).
#define RESPLE_PROF_PHASE_GPU(name) \
    RESPLE_NVTX(name); \
    RESPLE_PROF_GPU(name)
#define RESPLE_PROF_PHASE_CPU(name) \
    RESPLE_NVTX(name); \
    RESPLE_PROF_CPU(name)
