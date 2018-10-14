cmake_minimum_required(VERSION 3.5)
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

find_package(OpenCV)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
)

## Set up include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Require C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Altitude node
add_executable(altitude src/altitude.cpp)
target_link_libraries(altitude ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Mono odom node
add_executable(orca_mono_odom src/mono_odometry.cpp)
target_link_libraries(orca_mono_odom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Stereo odom node
add_executable(orca_stereo_odom src/stereo_odometry.cpp src/stereo_image.cpp src/util.cpp src/test.cpp)
target_link_libraries(orca_stereo_odom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
