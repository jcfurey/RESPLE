// Configure-time guard for the Eigen alignment pin in ../CMakeLists.txt.
//
// EIGEN_MALLOC_ALREADY_ALIGNED=1 routes Eigen's aligned_malloc to plain
// std::malloc, which glibc guarantees only 16-byte aligned. If Eigen's
// EIGEN_MAX_ALIGN_BYTES is larger than that (it becomes 32 or 64 under
// -march=native), Eigen's evaluators emit ALIGNED AVX stores (vmovapd) into
// that storage and the process segfaults inside the IEKF.
//
// The pin must therefore actually resolve to 16. Two previous attempts set a
// macro Eigen redefines unconditionally and were silently discarded; this
// file exists so a third cannot be.
#include <eigen3/Eigen/Core>

static_assert(EIGEN_MAX_ALIGN_BYTES == 16,
              "EIGEN_MAX_ALIGN_BYTES did not resolve to 16 — the alignment pin "
              "is inert. See the comment block in resple/CMakeLists.txt.");

// Eigen derives EIGEN_MAX_STATIC_ALIGN_BYTES from the same setting; if they
// disagree, fixed-size members and heap storage would use different rules.
static_assert(EIGEN_MAX_STATIC_ALIGN_BYTES <= 16,
              "EIGEN_MAX_STATIC_ALIGN_BYTES exceeds the heap alignment the "
              "allocator actually delivers.");

int main() { return 0; }
