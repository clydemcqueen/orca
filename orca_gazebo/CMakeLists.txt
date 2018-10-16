cmake_minimum_required(VERSION 3.5)
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
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Set up link directories
link_directories(${GAZEBO_LIBRARY_DIRS})

## Require C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

add_library(OrcaIMUPlugin src/imu_plugin.cpp)
add_dependencies(OrcaIMUPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(OrcaIMUPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(OrcaGroundTruthPlugin src/ground_truth_plugin.cpp)
add_dependencies(OrcaGroundTruthPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(OrcaGroundTruthPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(orca_gazebo_test src/test.cpp)
target_link_libraries(orca_gazebo_test ${catkin_LIBRARIES})
