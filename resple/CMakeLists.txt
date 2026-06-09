cmake_minimum_required(VERSION 3.16)
project(resple)

# Prefer target-based flags. Provide options to gate OpenMP/native arch.
option(ENABLE_OPENMP "Enable OpenMP parallelism" ON)
option(ENABLE_NATIVE_ARCH "Enable -march=native optimizations" ON)
option(ENABLE_CUDA "Enable CUDA k-NN acceleration (requires nvcc)" OFF)
# Sanitizer builds for the hardening workflow. Mutually exclusive; both force
# -O1 and drop -ffast-math so the sanitizer runtime sees the real control flow
# and FP semantics. Build with e.g.
#   colcon build --packages-select resple --cmake-args -DENABLE_TSAN=ON
# then run a bag under TSAN_OPTIONS=halt_on_error=0:second_deadlock_stack=1.
option(ENABLE_TSAN "Enable ThreadSanitizer (mutually exclusive with ASan)" OFF)
option(ENABLE_ASAN "Enable AddressSanitizer + UndefinedBehaviorSanitizer (mutually exclusive with TSan)" OFF)
# Debug-O0 build: -O0 -g, no native arch, no fast-math. Use when chasing
# heap corruption / SIGSEGV in __libc_free where -O3 inlining hides the
# real source line. Mutually exclusive with sanitizers (which set their
# own opt level). At -O0 the worker loop is too slow to keep up with
# 10 Hz scans — the async map-update lambda runs longer than 100 ms,
# the next worker cycle blocks on map_update_future_.wait() forever, and
# /localization/resple/odometry stops publishing. Use only for diagnosis.
option(ENABLE_DEBUG_O0 "Build with -O0 -g for crash-line resolution" OFF)
# Debug-O1 build: -O1 -g, no native arch. Enables basic optimizations
# (constant fold, simple inline of trivial getters) but skips vectorization,
# loop unrolling, and strict-aliasing assumptions — the most common
# UB-triggering optimizations. Used as a "safe enough" runtime when -O3
# crashes due to UB and -O0 is too slow to meet realtime deadlines.
option(ENABLE_DEBUG_O1 "Build with -O1 -g (UB-safe but near-release perf)" OFF)
# Debug-O3G build: -O3 -g3, full optimization with debug info. Use to keep
# the bug reproducing while also getting addr2line to resolve the exact
# source line. The crashing offset is at -O3 inlining boundaries; -g3
# preserves the symbol→line mapping inside inlined code.
option(ENABLE_DEBUG_O3G "Build with -O3 -g3 (production-speed crash with line numbers)" OFF)
if(ENABLE_TSAN AND ENABLE_ASAN)
	message(FATAL_ERROR "resple: ENABLE_TSAN and ENABLE_ASAN are mutually exclusive")
endif()
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Non-ROS deps not picked up by ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# OpenMP is optional and only required when ENABLE_OPENMP=ON
if(ENABLE_OPENMP)
	find_package(OpenMP REQUIRED)
endif()

# BLAS backend for Eigen (24x24 / 30x30 Cholesky in IEKF + general matrix
# products). Initially destabilized the simpler (I-KH)*P covariance update;
# now safe with Joseph-form posterior P=(I-KH)P(I-KH)^T+KRK^T.
find_package(BLAS QUIET)
if(BLAS_FOUND)
	add_definitions(-DEIGEN_USE_BLAS)
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# CUDA support — must precede add_library so .cu files compile correctly.
# CUDA 13 dropped Maxwell/Pascal: defaults below target Turing through Ada.
# For Jetson Nano (sm_53), build with CUDA <= 11 and override with
#   -DCMAKE_CUDA_ARCHITECTURES="53;75;86" or similar.
if(ENABLE_CUDA)
	enable_language(CUDA)
	if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
		# Turing (75), Ampere desktop (86), Orin (87), Ada Lovelace (89)
		set(CMAKE_CUDA_ARCHITECTURES 75 86 87 89)
	endif()
	message(STATUS "CUDA enabled: targeting compute ${CMAKE_CUDA_ARCHITECTURES}")
endif()

set(RESPLE_LIB_SOURCES
	src/Mapping.cpp
	src/RESPLE.cpp
	include/ikd-Tree/ikd_Tree.cpp
)
if(ENABLE_CUDA)
	list(APPEND RESPLE_LIB_SOURCES src/gpu/cuda_knn.cu)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED ${RESPLE_LIB_SOURCES})

if(ENABLE_CUDA)
	target_compile_definitions(${PROJECT_NAME} PUBLIC RESPLE_USE_CUDA)
	set_target_properties(${PROJECT_NAME} PROPERTIES
		CUDA_SEPARABLE_COMPILATION ON
	)
	# NVTX (Tools Extension): header-only API in CUDA Toolkit. We only need
	# the include path; the push/pop range API used in cuda_profile.h is
	# inlined and does not require linking libnvToolsExt. Disable with
	# -DENABLE_NVTX=OFF if a toolkit ships without nvtx3 headers.
	option(ENABLE_NVTX "Emit NVTX ranges for nsys profiling" ON)
	if(ENABLE_NVTX)
		find_path(NVTX3_INCLUDE_DIR nvtx3/nvToolsExt.h
			HINTS ${CUDAToolkit_INCLUDE_DIRS} /usr/local/cuda/include
			PATH_SUFFIXES include)
		if(NVTX3_INCLUDE_DIR)
			target_include_directories(${PROJECT_NAME} PRIVATE ${NVTX3_INCLUDE_DIR})
			target_compile_definitions(${PROJECT_NAME} PRIVATE RESPLE_NVTX_ENABLED)
			message(STATUS "NVTX ranges enabled (include: ${NVTX3_INCLUDE_DIR})")
		else()
			message(STATUS "NVTX headers not found; profile macros will skip nsys ranges")
		endif()
	endif()
endif()

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
)

# 2026-05-01 Eigen aligned-allocator ABI fix — second attempt.
#
# Production SIGSEGV (reproducible at -O3, addr2line resolved):
#   __libc_free  ←  Eigen::internal::handmade_aligned_free  (Memory.h:118)
#                ←  Eigen::internal::aligned_free            (Memory.h:206)
#                ←  Eigen::aligned_allocator::deallocate     (Memory.h:921)
#                ←  std::vector<PointXYZINormal,
#                              Eigen::aligned_allocator>::~vector()
#                ←  KD_TREE::Add_Points  (ikd_Tree.cpp:485, the
#                              `PointVector().swap(Downsample_Storage)` site)
#
# Free dispatches through `handmade_aligned_free`, which recovers the
# original malloc'd pointer by reading `ptr - sizeof(void*)`. If the matching
# allocation went through any other path (posix_memalign or _mm_malloc),
# those bytes are garbage and `std::free(garbage)` faults.
#
# First attempt (this commit's predecessor) tried pinning EIGEN_HAS_POSIX_MEMALIGN=1
# and EIGEN_HAS_MM_MALLOC=0 from the command line — DID NOT work, dispatch
# still hits handmade_aligned_free. Eigen's Memory.h defines those macros
# unconditionally (no #ifndef guard) based on its own feature-test of
# _GNU_SOURCE / _POSIX_ADVISORY_INFO, so a -D from the command line gets
# overwritten when Memory.h is preprocessed. _GNU_SOURCE is what we'd need
# to set instead of pinning the result macros.
#
# Second attempt (this block): two complementary defines.
#   _GNU_SOURCE                          : satisfies Eigen's posix_memalign
#                                          feature test inside Memory.h, so
#                                          its own #define of
#                                          EIGEN_HAS_POSIX_MEMALIGN evaluates
#                                          to 1 instead of 0.
#   EIGEN_MALLOC_ALREADY_ALIGNED=1       : safety net — tells Eigen to treat
#                                          std::malloc as already returning
#                                          16-byte-aligned memory (true on
#                                          x86_64 glibc). aligned_free then
#                                          short-circuits to std::free,
#                                          bypassing the handmade path
#                                          regardless of which other macros
#                                          Memory.h sets.
#   EIGEN_DEFAULT_ALIGN_BYTES=16         : pin alignment to 16 to match what
#                                          PCL was compiled with (per the
#                                          PCL build warning) and what
#                                          std::malloc actually delivers.
#                                          Prevents Eigen from over-aligning
#                                          types to 32 (AVX) when the
#                                          underlying malloc only guarantees
#                                          16, which would re-trigger the
#                                          aligned-malloc dispatch logic.
target_compile_definitions(${PROJECT_NAME} PUBLIC
	_GNU_SOURCE
	EIGEN_MALLOC_ALREADY_ALIGNED=1
	EIGEN_DEFAULT_ALIGN_BYTES=16
)

# Enforce C++17 and apply optimization flags to the library target
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# Warning set matches the workspace's colcon_defaults where practical. -Wshadow
# catches the local/member name reuse that the audit flagged as the kind of
# bug that would slip past code review. -Wextra is strictly more than -Wall.
# Kept private so downstream consumers aren't forced into the same warnings.
target_compile_options(${PROJECT_NAME} PRIVATE
	-Wall
	-Wextra
	-Wshadow
	$<$<CONFIG:Debug>:-DEIGEN_INITIALIZE_MATRICES_BY_NAN>
)

# Optimization / sanitizer flag selection.
#
# -ffast-math was previously enabled unconditionally. It was dropped because
# this library runs a 24x24 / 30x30 Joseph-form Kalman update whose
# PSD-preservation argument depends on IEEE-754 associativity semantics; with
# -ffast-math the compiler is free to reassociate the (I-KH)P(I-KH)^T + KRK^T
# expression and break that guarantee, and all NaN/Inf divergence checks
# become no-ops. -O3 -march=native still applies.
#
# Sanitizers take precedence over the default -O3 path: they need -O1 (so the
# stack is walkable in reports) and drop -march=native reliance.
if(ENABLE_TSAN)
	set(RESPLE_OPT_FLAGS -O1 -g -fno-omit-frame-pointer)
	set(RESPLE_SAN_FLAGS -fsanitize=thread)
	message(STATUS "resple: ThreadSanitizer enabled; optimizations reduced to -O1")
elseif(ENABLE_ASAN)
	set(RESPLE_OPT_FLAGS -O1 -g -fno-omit-frame-pointer)
	set(RESPLE_SAN_FLAGS -fsanitize=address -fsanitize=undefined)
	message(STATUS "resple: AddressSanitizer + UBSan enabled; optimizations reduced to -O1")
elseif(ENABLE_DEBUG_O0)
	# -O0 keeps every line on its own; -g3 emits maximum debug info so
	# addr2line resolves source line numbers cleanly. Drop -march=native
	# to remove SIMD-induced inlining variance.
	set(RESPLE_OPT_FLAGS -O0 -g3 -fno-omit-frame-pointer)
	message(STATUS "resple: Debug-O0 build for crash-line resolution")
elseif(ENABLE_DEBUG_O1)
	# Basic optimization, no aggressive transforms. -fno-strict-aliasing
	# disables type-based alias analysis (a common UB-trigger if any code
	# punning is happening). -fno-tree-vectorize disables auto-vectorization
	# (the other common UB-trigger when code reads/writes via mismatched
	# alignment). Result: closer to release perf than -O0 but with the two
	# UB-exposing classes of optimization disabled.
	set(RESPLE_OPT_FLAGS -O1 -g -fno-omit-frame-pointer
	                     -fno-strict-aliasing -fno-tree-vectorize)
	message(STATUS "resple: Debug-O1 build (UB-safe but near-release perf)")
elseif(ENABLE_DEBUG_O3G)
	# Production-speed binary WITH debug info. Same -O3 inlining/vectorization
	# that triggers the production SIGSEGV is preserved, but -g3 emits the
	# debug info needed for addr2line to resolve the exact source line at
	# Add_Points + 0xNNN. Use this when you need to localize a -O3-only crash
	# without changing optimization level.
	set(RESPLE_OPT_FLAGS -O3 -g3 -fno-omit-frame-pointer)
	message(STATUS "resple: -O3 -g3 build for crash-line resolution at production speed")
elseif(ENABLE_NATIVE_ARCH)
	set(RESPLE_OPT_FLAGS -O3 -march=native)
else()
	set(RESPLE_OPT_FLAGS -O3)
endif()
target_compile_options(${PROJECT_NAME} PRIVATE ${RESPLE_OPT_FLAGS})
if(RESPLE_SAN_FLAGS)
	# Sanitizer flags must apply to every TU and to the link of any executable
	# that pulls in this library, so mark them PUBLIC.
	target_compile_options(${PROJECT_NAME} PUBLIC ${RESPLE_SAN_FLAGS})
	target_link_options(${PROJECT_NAME} PUBLIC ${RESPLE_SAN_FLAGS})
endif()

# Executables are thin main() wrappers that forward to the library's exported
# entry functions (respleMain / mappingMain). The heavy node sources live only
# in ${PROJECT_NAME}, so they're compiled once instead of twice.
add_executable(Mapping src/Mapping_main.cpp)
add_executable(RESPLE src/RESPLE_main.cpp)

# -rdynamic exports symbols in the dynamic symbol table so the crash-handler's
# backtrace_symbols_fd() in RESPLE.cpp prints function names instead of just
# addresses. Tiny binary-size cost; worth it every time RESPLE segfaults.
target_link_options(RESPLE  PRIVATE -rdynamic)
target_link_options(Mapping PRIVATE -rdynamic)

# ROS deps are linked into ${PROJECT_NAME} by ament_auto_add_library via package.xml.
# Use plain signature consistently (ament_target_dependencies uses plain internally).
# The thin executables only need the library; PCL/YAML/OpenMP propagate
# transitively from it (plain-signature links are usage requirements).
target_link_libraries(${PROJECT_NAME}
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

target_link_libraries(RESPLE  ${PROJECT_NAME})
target_link_libraries(Mapping ${PROJECT_NAME})

if(ENABLE_OPENMP)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

if(BLAS_FOUND)
  target_link_libraries(${PROJECT_NAME} ${BLAS_LIBRARIES})
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(TARGETS RESPLE
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS Mapping
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Dependency-light estimator-core unit tests (Eigen + GoogleTest only — no
  # ROS 2 / PCL). The same sources also build standalone without colcon via
  # resple/test/CMakeLists.txt. Each entry is guarded with EXISTS so the commit
  # series stays bisectable while tests are added incrementally.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 QUIET)
  set(RESPLE_UNIT_TESTS
    test/test_geometry_core.cpp
    test/test_point_cloud_adapter.cpp)
  foreach(_test_src ${RESPLE_UNIT_TESTS})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${_test_src})
      get_filename_component(_test_name ${_test_src} NAME_WE)
      ament_add_gtest(${_test_name} ${_test_src})
      target_include_directories(${_test_name} PRIVATE include)
      if(TARGET Eigen3::Eigen)
        target_link_libraries(${_test_name} Eigen3::Eigen)
      endif()
    endif()
  endforeach()
endif()

ament_auto_package()
