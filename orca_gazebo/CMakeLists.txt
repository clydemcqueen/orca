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

add_library(OrcaDragPlugin src/drag_plugin.cpp)
add_dependencies(OrcaDragPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(OrcaDragPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(OrcaThrusterPlugin src/thruster_plugin.cpp)
add_dependencies(OrcaThrusterPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(OrcaThrusterPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(OrcaBarometerPlugin src/barometer_plugin.cpp)
add_dependencies(OrcaBarometerPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(OrcaBarometerPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(OrcaGPSPlugin src/gps_plugin.cpp)
add_dependencies(OrcaGPSPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(OrcaGPSPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(OrcaBuoyancyPlugin src/buoyancy_plugin.cpp)
add_dependencies(OrcaBuoyancyPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(OrcaBuoyancyPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})