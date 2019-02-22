cmake_minimum_required(VERSION 2.8.3)
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

find_package(casadi REQUIRED)

find_library(GLOG glog)

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

message("ceres include: " ${CERES_INCLUDE_DIRS})
message("ceres library: " ${CERES_LIBRARIES})

#MESSAGE(STATUS "yt:=  " ${catkin_INCLUDE_DIRS})

add_library(ncopt_lib
        src/non_constrained_optimiztion/line_search_direction.cpp
        src/non_constrained_optimiztion/line_search_step_length.cpp
        src/non_constrained_optimiztion/gradient_problem_solve.cpp
        src/non_constrained_optimiztion/line_search_minimizer.cpp
        src/non_constrained_optimiztion/minimize.cpp
        src/non_constrained_optimiztion/type.cpp)
target_link_libraries(ncopt_lib
        ${EIGEN3_LIBRARIES}
        ${GLOG})

add_library(path_smoothing
        include/path_smoothing/path_smoothing.hpp
        src/path_smoothing/evaluator.cpp)
target_link_libraries(path_smoothing
        ncopt_lib
        casadi
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(path_smoothing_demo
        test/test_path_smoothing.cpp)
target_link_libraries(path_smoothing_demo
        path_smoothing
        ${catkin_LIBRARIES})

#add_executable(steepest_decent_demo
#        test/test_steepest_decent.cpp)
#target_link_libraries(steepest_decent_demo
#        ncopt_lib
#        ${CERES_LIBRARIES})


#add_library(${PROJECT_NAME}
#  src/cg_solver.cpp
#  src/path_smoothing.cpp)
#target_link_libraries(${PROJECT_NAME}
#  ${CERES_LIBRARIES})

add_library(chomp_optimizer
        src/chomp.cpp)
target_link_libraries(chomp_optimizer
        ${catkin_LIBRARIES})

#add_executable(cg_demo
#  src/path_smooth_demo_node.cpp)
#
#target_link_libraries(cg_demo
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${PROJECT_NAME})

#add_executable(chomp_demo
#  src/drivable_region_node.cpp)

#target_link_libraries(chomp_demo
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  chomp_optimizer)


