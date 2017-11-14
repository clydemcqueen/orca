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
)

## Set up include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Require C++14
add_compile_options(-std=c++14)

## Base
add_executable(orca_base src/orca_base.cpp)
target_link_libraries(orca_base ${catkin_LIBRARIES})