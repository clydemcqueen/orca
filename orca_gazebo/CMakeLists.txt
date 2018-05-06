cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0054 OLD) # Gets rid of an annoying warning in CMake 3.5.1, but need to fix one day
project(orca_gazebo)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  orca_base
  orca_msgs
  roscpp
  rospy
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS 
    gazebo_ros
    orca_base
    orca_msgs
    roscpp
    rospy
)

## Set up include directories
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Set up link directories
link_directories(${GAZEBO_LIBRARY_DIRS})

## Require C++14
add_compile_options(-std=c++14)

add_library(DragPlugin src/drag_plugin.cpp)
add_dependencies(DragPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(DragPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ThrusterPlugin src/thruster_plugin.cpp)
add_dependencies(ThrusterPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(ThrusterPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(BarometerPlugin src/barometer_plugin.cpp)
add_dependencies(BarometerPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(BarometerPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
