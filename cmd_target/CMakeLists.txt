cmake_minimum_required(VERSION 3.0.2)
project(cmd_target)

find_package(catkin REQUIRED COMPONENTS
  gary_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmd_target
#  CATKIN_DEPENDS gary_msgs geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../../devel/include
  ../../../devel/include
)

add_executable(
  cmd_target
  src/cmd_target_node.cpp
  src/cmd_target.cpp
)

target_link_libraries(
  cmd_target
  ${catkin_LIBRARIES}
)