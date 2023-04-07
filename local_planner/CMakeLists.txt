cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ../../devel/include
    ../../../devel/include
)

## Declare executables
add_executable(
    localPlanner 
    src/local_planner_node.cpp
)
add_executable(
    pathFollower 
    src/path_follower_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
    localPlanner 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)
target_link_libraries(
    pathFollower 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)