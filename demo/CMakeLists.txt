if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "CMAKE BUILD TYPE: " ${CMAKE_BUILD_TYPE})
  add_definitions(-DDEBUG)
endif ()

set(demo_deps
  roslib
  roscpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  grid_map_ros
  grid_map_cv)

find_package(catkin REQUIRED COMPONENTS
  ${demo_deps})
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(smooth_without_obstacle_demo
  smooth_without_obstacle_demo.cpp)
target_link_libraries(smooth_without_obstacle_demo
  path_smoothing
  ${catkin_LIBRARIES})

add_executable(smooth_with_obstacle_demo
  smooth_with_obstacle_demo.cpp)
add_dependencies(smooth_with_obstacle_demo ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(smooth_with_obstacle_demo
  path_smoothing
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(steepest_decent_demo
  test_steepest_decent.cpp)
target_link_libraries(steepest_decent_demo
  ncopt_lib
  ${CERES_LIBRARIES})



