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
  opt_utils
  roslib
  roscpp
  nav_msgs
  geometry_msgs
  internal_grid_map
  visualization_msgs
  rosparam_handler
  dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps})
find_package(Boost COMPONENTS filesystem serialization thread regex timer REQUIRED)
find_package(Ceres REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gpmp2 QUIET)
find_package(GTSAM QUIET)

if (${GTSAM_FOUND} AND ${gpmp2_FOUND})
  add_definitions(-DGPMP2_SMOOTHING_ENABLE)
  message("enable gpmp2 smoothing method")
endif ()

generate_ros_parameter_files(
  cfg/smoothing_demo.params)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_deps}
  DEPENDS Ceres casadi
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_FILESYSTEM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${gpmp2_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)
aux_source_directory(src/non_constrained_optimiztion ncopt_src)
add_library(ncopt_lib
  include/non_constrained_optimiztion/type.hpp
  include/non_constrained_optimiztion/gradient_problem_options.hpp
  ${ncopt_src})
target_link_libraries(ncopt_lib
  ${EIGEN3_LIBRARIES}
  glog)

add_library(path_smoothing
  include/path_smoothing/path_smoothing.hpp
  include/path_smoothing/distance_function.hpp
  src/path_smoothing/cg_smoothing_function.cpp
  src/path_smoothing/path_smoothing.cpp)
target_link_libraries(path_smoothing
  ncopt_lib
  casadi
  gpmp2
  ${CERES_LIBRARIES}
  ${catkin_LIBRARIES})


add_executable(smooth_without_obstacle_demo
  demo/smooth_without_obstacle_demo.cpp)
target_link_libraries(smooth_without_obstacle_demo
  path_smoothing
  ${catkin_LIBRARIES})
#
add_executable(smooth_with_obstacle_demo
  demo/smooth_with_obstacle_demo.cpp)
add_dependencies(smooth_with_obstacle_demo ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(smooth_with_obstacle_demo
  path_smoothing
  ${catkin_LIBRARIES})

add_executable(steepest_decent_demo
  demo/test_steepest_decent.cpp)
target_link_libraries(steepest_decent_demo
  ncopt_lib
  ${CERES_LIBRARIES})



