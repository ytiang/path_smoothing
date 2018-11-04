cmake_minimum_required(VERSION 2.8.3)
project(path_smoothing)
set(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(catkin_deps
  opt_utils
  roslib
  roscpp
  nav_msgs
  geometry_msgs
  interactive_markers
  internal_grid_map
  visualization_msgs
  dynamic_reconfigure
  rosparam_handler
  space_explore
)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps}
)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 QUIET)
find_package(Ceres REQUIRED)

generate_ros_parameter_files(config/param.cfg)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_deps}
#  DEPENDS OpenCV
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
)

MESSAGE(STATUS "yt:=  " ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/cg_solver.cpp
  src/path_smoothing.cpp)
target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES})

add_library(chomp_optimizer
  src/chomp.cpp)
target_link_libraries(chomp_optimizer
  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
  src/drivable_region_node.cpp)
add_dependencies(${PROJECT_NAME}_node  ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
#  ${PROJECT_NAME}
  chomp_optimizer)


