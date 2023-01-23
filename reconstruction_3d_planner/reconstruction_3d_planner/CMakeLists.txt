cmake_minimum_required(VERSION 2.8.3)
project(reconstruction_3d_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  octomap_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  trajectory_msgs
)

add_service_files(
  FILES
  OptimizeCurrentPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 3d_reconstruction_planner
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    move_base_msgs
    nav_msgs
    octomap_msgs
    roscpp
    rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(planner
  src/planner.cpp
)
add_dependencies(planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(planner
  ${catkin_LIBRARIES}
)
