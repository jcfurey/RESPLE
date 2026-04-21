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
if(ENABLE_TSAN AND ENABLE_ASAN)
	message(FATAL_ERROR "resple: ENABLE_TSAN and ENABLE_ASAN are mutually exclusive")
endif()
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(estimate_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)
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

add_library(${PROJECT_NAME} ${RESPLE_LIB_SOURCES})

if(ENABLE_CUDA)
	target_compile_definitions(${PROJECT_NAME} PUBLIC RESPLE_USE_CUDA)
	set_target_properties(${PROJECT_NAME} PROPERTIES
		CUDA_SEPARABLE_COMPILATION ON
	)
endif()

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
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

# Executables just wrap the library - no source duplication
add_executable(Mapping)
target_sources(Mapping PRIVATE src/Mapping.cpp)

add_executable(RESPLE)
target_sources(RESPLE PRIVATE src/RESPLE.cpp)

# -rdynamic exports symbols in the dynamic symbol table so the crash-handler's
# backtrace_symbols_fd() in RESPLE.cpp prints function names instead of just
# addresses. Tiny binary-size cost; worth it every time RESPLE segfaults.
target_link_options(RESPLE  PRIVATE -rdynamic)
target_link_options(Mapping PRIVATE -rdynamic)

ament_target_dependencies(${PROJECT_NAME} 
	rclcpp
	rclcpp_components
	std_msgs
	geometry_msgs
	nav_msgs
	sensor_msgs
	visualization_msgs
	std_srvs
	livox_interfaces
	livox_ros_driver
	livox_ros_driver2
	estimate_msgs
	pcl_conversions
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
	tf2_sensor_msgs
	diagnostic_updater
	rclcpp_action
	rclcpp_lifecycle
	rosbag2_cpp
)

# Use plain signature consistently (ament_target_dependencies uses plain internally)
target_link_libraries(${PROJECT_NAME}
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

target_link_libraries(RESPLE
	${PROJECT_NAME}
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

target_link_libraries(Mapping
	${PROJECT_NAME}
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

if(ENABLE_OPENMP)
  target_link_libraries(RESPLE OpenMP::OpenMP_CXX)
  target_link_libraries(Mapping OpenMP::OpenMP_CXX)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

if(BLAS_FOUND)
  target_link_libraries(${PROJECT_NAME} ${BLAS_LIBRARIES})
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
# Register components for composition
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "RESPLE"
  EXECUTABLE RESPLE_node
)

install(TARGETS RESPLE
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS Mapping
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Export only ament packages (yaml-cpp is not an ament package)
ament_export_dependencies(
	rclcpp
	livox_ros_driver
	livox_ros_driver2
	livox_interfaces
	diagnostic_updater
	rclcpp_action
	rclcpp_lifecycle
	rosbag2_cpp
)
ament_export_libraries(${PROJECT_NAME})
ament_package()
