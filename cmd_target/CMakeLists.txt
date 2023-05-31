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

find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmd_target
#  CATKIN_DEPENDS gary_msgs geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
file(GLOB PRO_FORM_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB PRO_RESOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource/*.qrc)
file(GLOB_RECURSE PRO_INCLUDE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h )
file(GLOB_RECURSE PRO_SOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../../devel/include
  ../../../devel/include
)

qt5_wrap_ui(QT_UI_HPP ${PRO_FORM_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOURCES_CPP ${PRO_RESOURCES_DIR})
add_executable(${PROJECT_NAME}
        ${PRO_SOURCES_DIR}
        ${QT_RESOURCES_CPP}
        ${PRO_INCLUDE_DIR}
        ${QT_MOC_HPP}
        ${PRO_FORM_DIR}
)

target_link_libraries(
  cmd_target
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

target_link_libraries(
  cmd_target
  /usr/lib/x86_64-linux-gnu/libtiff.so.5
  /usr/lib/x86_64-linux-gnu/libsqlite3.so.0
)
