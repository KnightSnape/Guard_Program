cmake_minimum_required(VERSION 3.0.2)
project(guard_simulator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  rospack
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES guard_simulator
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp roslib rospack sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ../../devel/include
  ../../../devel/include
)

add_executable(
  guard_simulator
  src/guard_simulator_node.cpp
  src/guard_simulator.cpp
)

target_link_libraries(
  guard_simulator
  ${catkin_LIBRARIES}
)