cmake_minimum_required(VERSION 2.8.3)
project(orca_base)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  message_generation
  pid
  roscpp
  rospy
  roslint
  sensor_msgs
  std_msgs
  tf
)

## Generate code for messages we define
add_message_files(DIRECTORY msg FILES
  Camera.msg
  Depth.msg
  Lights.msg
  Thruster.msg
)
generate_messages()

## Declare a catkin package and export dependencies
catkin_package(CATKIN_DEPENDS message_runtime)

## Set up include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Require C++14
add_compile_options(-std=c++14)

## Lint everything
roslint_cpp()

## Driver -- TODO cross-compile for armf
add_executable(orca_driver src/orca_driver.cpp)
target_link_libraries(orca_driver ${catkin_LIBRARIES})
add_dependencies(orca_driver ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Base -- TODO split into library and node
add_executable(orca_base src/orca_base.cpp)
target_link_libraries(orca_base ${catkin_LIBRARIES})
add_dependencies(orca_base ${${PROJECT_NAME}_EXPORTED_TARGETS})