cmake_minimum_required(VERSION 2.8.3)
project(orca_topside)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  orca_msgs
  roscpp
  rviz
)

## Declare a catkin package
catkin_package()

## Set up include directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Require C++14
add_compile_options(-std=c++14)

## Topside includes an rviz plugin, so we need to include Qt and Ogre

set(CMAKE_AUTOMOC ON)

## Qt4 or 5?
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

## Add include file here; this works around a build problem w/ Qt MOC
add_library(orca_topside src/orca_panel.cpp include/orca_topside/orca_panel.h)

target_link_libraries(orca_topside ${QT_LIBRARIES} ${catkin_LIBRARIES})
