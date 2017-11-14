cmake_minimum_required(VERSION 2.8.3)
project(orca_msgs)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS message_generation)

## Generate code for messages we define
add_message_files(DIRECTORY msg FILES
  Barometer.msg
  Battery.msg
  Camera.msg
  Leak.msg
  Lights.msg
  Proc.msg
  Thrusters.msg
)
generate_messages()

## Declare a catkin package and export dependencies
catkin_package(CATKIN_DEPENDS message_runtime)

## Set up include directories
include_directories(${catkin_INCLUDE_DIRS})
