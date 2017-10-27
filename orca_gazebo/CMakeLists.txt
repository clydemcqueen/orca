cmake_minimum_required(VERSION 2.8.3)
project(orca_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  orca_msgs
  roscpp
  rospy
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## Declare downstream dependencies
catkin_package(
  CATKIN_DEPENDS 
    gazebo_ros
    orca_msgs
    roscpp
    rospy
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Require C++14
add_compile_options(-std=c++14)

add_library(ThrusterPlugin src/thruster_plugin.cpp)
target_link_libraries(ThrusterPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(ThrusterPlugin ${catkin_EXPORTED_TARGETS})

add_library(BarometerPlugin src/barometer_plugin.cpp)
target_link_libraries(BarometerPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(BarometerPlugin ${catkin_EXPORTED_TARGETS})