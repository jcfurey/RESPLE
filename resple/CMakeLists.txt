cmake_minimum_required(VERSION 3.16)
project(resple)

# Prefer target-based flags. Provide options to gate OpenMP/native arch.
option(ENABLE_OPENMP "Enable OpenMP parallelism" ON)
option(ENABLE_NATIVE_ARCH "Enable -march=native optimizations" OFF)
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
# OpenMP is optional and only required when ENABLE_OPENMP=ON
if(ENABLE_OPENMP)
	find_package(OpenMP REQUIRED)
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(${PROJECT_NAME}
		src/Mapping.cpp
		src/RESPLE.cpp
		include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
)

# Enforce C++17 and apply optimization flags to the library target
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE 
	-Wall
	$<$<CONFIG:Debug>:-DEIGEN_INITIALIZE_MATRICES_BY_NAN>
)
if(ENABLE_NATIVE_ARCH)
	target_compile_options(${PROJECT_NAME} PRIVATE -O3 -march=native)
else()
	target_compile_options(${PROJECT_NAME} PRIVATE -O3)
endif()

# Executables just wrap the library - no source duplication
add_executable(Mapping)
target_sources(Mapping PRIVATE src/Mapping.cpp)

add_executable(RESPLE)
target_sources(RESPLE PRIVATE src/RESPLE.cpp)

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
)
ament_export_libraries(${PROJECT_NAME})
ament_package()
