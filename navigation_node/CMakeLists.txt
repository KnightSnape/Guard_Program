cmake_minimum_required(VERSION 3.0.2)
project(navigation_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  rospack
  roslib
  tf2_ros
  tf2_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES navigation_node
   CATKIN_DEPENDS roscpp sensor_msgs nav_msgs rospack roslib
#  DEPENDS system_lib

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ../../devel/include
  ../../../devel/include
)

add_executable(
  navigation_node
  src/navigation_node.cpp
)

target_link_libraries(
  navigation_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
