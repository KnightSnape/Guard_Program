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
  rospack
  roslib
  roslaunch
)

find_package(ngraph REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  include
  include/behavior
  include/executor
  include/utility
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ../../devel/include
  ../../../devel/include
)

add_executable(
  armor_decision
  src/decision.cpp
  src/decision_tree.cpp
  include/behavior/autoaim_switch.cpp
  include/behavior/chassic_switch.cpp
  include/behavior/game_start.cpp
  include/behavior/gimbal_switch.cpp
  include/behavior/race_choose.cpp
  include/executor/autoaim_executor.cpp
  include/executor/chassis_executor.cpp
  include/executor/gimbal_executor.cpp
  include/utility/point_graph.cpp
  include/utility/pos_manager.cpp
  include/utility/blackboard.cpp
  include/Inference/decisionInference.cpp
  include/Inference/freeInference.cpp
)

target_link_libraries(
  armor_decision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${InferenceEngine_LIBRARIES}
  ${ngraph_LIBRARIES}
)
