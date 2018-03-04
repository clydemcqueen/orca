cmake_minimum_required(VERSION 2.8.3)
project(orca_vision)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  gscam
  image_proc
  stereo_image_proc
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
  gscam
  image_proc
  stereo_image_proc
)

## Set up include directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Require C++14
add_compile_options(-std=c++14)