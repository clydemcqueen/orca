#!/usr/bin/env bash

# Start orca ROS nodes

# To install:           sudo cp orca_ros.service /lib/systemd/system
# To start:             sudo systemctl start orca_ros.service
# To stop:              sudo systemctl stop orca_ros.service
# To start on boot:     sudo systemctl enable orca_ros.service
# To not start on boot: sudo systemctl disable orca_ros.service

bash -c "source /home/pi/orca_catkin_ws/devel/setup.bash && roslaunch orca_driver sub.launch"
