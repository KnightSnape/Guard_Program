cmake_minimum_required(VERSION 3.0.2)
project(armor_decision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  include/behavior
  include/executor
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ../../devel/include
  ../../../devel/include
)

add_executable(
  armor_decision
  src/decision.cpp
  src/decision_tree.cpp
  src/decision.hpp
  include/decision_tree.h
  include/blackboard.hpp
  include/Timer.h
)

target_link_libraries(
  armor_decision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
