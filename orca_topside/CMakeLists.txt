cmake_minimum_required(VERSION 3.5)
project(orca_topside)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  orca_base
  orca_msgs
  roscpp
  rospy
  rviz
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
  orca_base
  orca_msgs
  roscpp
  rospy
)

## Set up include directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Require C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)

## Qt4 or 5?
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  find_package(QtGStreamer REQUIRED)
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  find_package(Qt5GStreamer REQUIRED)
endif()

## More include directories
include_directories(${QTGSTREAMER_INCLUDES})

## Qt MOC can't find .h files if they're in a different directory; as a work-around add them here
add_library(orca_topside
  src/orca_panel.cpp
  src/video_panel.cpp
  include/orca_topside/orca_panel.h
  include/orca_topside/video_panel.h)
add_dependencies(orca_topside ${catkin_EXPORTED_TARGETS})
target_link_libraries(orca_topside ${QT_LIBRARIES} ${QTGSTREAMER_UI_LIBRARIES} ${catkin_LIBRARIES})
