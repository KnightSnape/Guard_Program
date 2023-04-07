cmake_minimum_required(VERSION 3.0.2)
project(gary_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  DR16Receiver.msg
  GameStatus.msg
  GameResult.msg
  RobotHP.msg
  ICRABuffDebuffZoneAndLurkStatus.msg
  FieldEvents.msg
  SupplyProjectileAction.msg
  SupplyProjectileRequest.msg
  RefereeWarning.msg
  DartRemainingTime.msg
  RobotStatus.msg
  PowerHeat.msg
  RobotPosition.msg
  RobotBuff.msg
  AerialRobotEnergy.msg
  RobotHurt.msg
  ShootData.msg
  BulletRemaining.msg
  RFIDStatus.msg
  DartClientCmd.msg
  InteractiveData.msg
  CustomController.msg
  ClientCommand.msg
  ClientReceive.msg
  ImageTransmitter.msg
  AutoAIM.msg
  PID.msg
  DualLoopPID.msg
  DualLoopPIDWithFilter.msg
  InteractiveDataRecv.msg
  InteractiveDataSend.msg
  PIDWithFilter.msg
)

add_service_files(
  FILES
  VisionModeSwitch.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


