cmake_minimum_required(VERSION 2.8)
project(path_smoothing)
set(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "CMAKE BUILD TYPE: " ${CMAKE_BUILD_TYPE})
  add_definitions(-DDEBUG)
endif ()

set(catkin_deps
  rosparam_handler
  dynamic_reconfigure
  grid_map_core)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps})
find_package(Ceres REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED)

find_package(gpmp2 QUIET)
find_package(GTSAM QUIET)
message("**********gtsam include: ${GTSAM_INCLUDE_DIR}, libraries: ${GTSAM_LIBRARY}")

find_package(PythonLibs 2.7 REQUIRED)

if (${GTSAM_FOUND} AND ${gpmp2_FOUND})
  find_package(Boost COMPONENTS filesystem serialization thread regex timer REQUIRED)
  add_definitions(-DGPMP2_SMOOTHING_ENABLE)
  set(GTSAM_LIBRARIES "gtsam")
  set(gpmp2_LIBRARIES "gpmp2")
  message("enable gpmp2 smoothing method")
endif ()

generate_ros_parameter_files(
  cfg/smoothing_demo.params)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ncopt_lib
  CATKIN_DEPENDS ${catkin_deps}
  DEPENDS CERES CASADI EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${gpmp2_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${CASADI_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
)
aux_source_directory(src/non_constrained_optimiztion ncopt_src)
add_library(ncopt_lib
  include/non_constrained_optimiztion/type.hpp
  include/non_constrained_optimiztion/gradient_problem_options.hpp
  ${ncopt_src})
target_link_libraries(ncopt_lib
  glog
  ${PYTHON_LIBRARIES})

add_library(tiny_spline_lib
  src/tiny_spline/tinysplinecpp.cpp
  src/tiny_spline/tinyspline.c)

add_library(path_smoothing
  src/path_smoothing/cg_smoothing_function.cpp
  src/path_smoothing/path_smoothing.cpp
  src/path_smoothing/path_smoothing_unit.cpp
  )
target_link_libraries(path_smoothing
  ncopt_lib
  ${CASADI_LIBRARIES}
  ${gpmp2_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${CERES_LIBRARIES}
  ${catkin_LIBRARIES}
  tiny_spline_lib
  )
#
add_subdirectory(demo)



