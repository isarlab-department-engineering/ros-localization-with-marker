cmake_minimum_required(VERSION 2.8.3)
project(marker_localization_trajectory)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  aruco_detection
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_follower src/trajectory-follower.cpp)

add_dependencies(trajectory_follower aruco_detection_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
   trajectory_follower
   ${catkin_LIBRARIES}
)

