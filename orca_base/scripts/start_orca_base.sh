#!/usr/bin/env bash

# Start orca ROS base

# To install:           sudo cp orca_base.service /lib/systemd/system
# To start:             sudo systemctl start orca_base.service
# To stop:              sudo systemctl stop orca_base.service
# To start on boot:     sudo systemctl enable orca_base.service
# To not start on boot: sudo systemctl disable orca_base.service

bash -c "source /home/pi/orca_catkin_ws/devel/setup.bash && stdbuf -o L roslaunch orca_base base.launch"
