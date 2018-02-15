#!/usr/bin/env bash

# Start orca ROS topside
bash -c "source ~/orca_catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://orca_wifi:11311 && rqt &"
bash -c "source ~/orca_catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://orca_wifi:11311 && roslaunch orca_topside topside.launch"
