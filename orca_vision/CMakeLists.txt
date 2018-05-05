cmake_minimum_required(VERSION 2.8.3)
project(orca_vision)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gscam
  image_proc
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  stereo_image_proc
  visualization_msgs
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
)

## Set up include directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Require C++14
add_compile_options(-std=c++14)

## Altitude node
add_executable(altitude src/altitude.cpp)
target_link_libraries(altitude ${catkin_LIBRARIES})

## Mono odom node
add_executable(orca_mono_odom src/mono_odometry.cpp)
target_link_libraries(orca_mono_odom ${catkin_LIBRARIES})

## Stereo odom node
add_executable(orca_stereo_odom src/stereo_odometry.cpp src/stereo_image.cpp src/util.cpp src/test.cpp)
target_link_libraries(orca_stereo_odom ${catkin_LIBRARIES})
