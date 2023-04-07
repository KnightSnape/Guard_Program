cmake_minimum_required(VERSION 3.0.2)
project(armor_decision)

set(ngraph_DIR "/opt/intel/openvino_2022/runtime/cmake")
set(InferenceEngine_DIR "/opt/intel/openvino_2022/runtime/cmake")

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

find_package(ngraph REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  include/behavior
  include/executor
  include/utility
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
  include/utility/point_graph.cpp
)

target_link_libraries(
  armor_decision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${InferenceEngine_LIBRARIES}
  ${ngraph_LIBRARIES}
)
