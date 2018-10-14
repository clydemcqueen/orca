cmake_minimum_required(VERSION 3.5)
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
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Driver
add_executable(orca_driver src/orca_driver.cpp src/maestro.cpp)
add_dependencies(orca_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(orca_driver ${catkin_LIBRARIES})
