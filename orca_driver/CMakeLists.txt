cmake_minimum_required(VERSION 2.8.3)
project(orca_driver)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  orca_msgs
  roscpp
  rospy
  std_msgs
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
    orca_msgs
    roscpp
    rospy
    std_msgs
)

## Set up include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Require C++14
add_compile_options(-std=c++14)

## Driver
add_executable(orca_driver src/orca_driver.cpp src/maestro.cpp)
add_dependencies(orca_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(orca_driver ${catkin_LIBRARIES})
