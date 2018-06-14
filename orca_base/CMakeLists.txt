cmake_minimum_required(VERSION 2.8.3)
project(orca_base)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  orca_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
    orca_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
  INCLUDE_DIRS include
)

## Set up include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Require C++14
add_compile_options(-std=c++14)

## Avoid warnings when we use visualization_msgs::Marker::DELETE and visualization_msgs::Marker::ADD in a conditional expression
add_compile_options(-Wno-enum-compare)

## Base
add_executable(orca_base src/orca_base.cpp src/orca_motion.cpp src/orca_mission.cpp)
add_dependencies(orca_base ${catkin_EXPORTED_TARGETS})
target_link_libraries(orca_base ${catkin_LIBRARIES})

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
