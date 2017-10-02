cmake_minimum_required(VERSION 2.8.3)
project(orca_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_ros
  orca_base
  orca_description
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Require C++14
add_compile_options(-std=c++14)

add_library(ThrusterPlugin src/thruster_plugin.cpp) # TODO SHARED?
target_link_libraries(ThrusterPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(ThrusterPlugin ${catkin_EXPORTED_TARGETS})

add_library(DepthPlugin src/depth_plugin.cpp) # TODO SHARED?
target_link_libraries(DepthPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(DepthPlugin ${catkin_EXPORTED_TARGETS})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    orca_base
    orca_description
)