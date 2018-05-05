#!/usr/bin/env bash

# Start orca ROS driver

# To install:           sudo cp orca_driver.service /lib/systemd/system
# To start:             sudo systemctl start orca_driver.service
# To stop:              sudo systemctl stop orca_driver.service
# To start on boot:     sudo systemctl enable orca_driver.service
# To not start on boot: sudo systemctl disable orca_driver.service

bash -c "source /home/pi/orca_catkin_ws/devel/setup.bash && stdbuf -o L roslaunch orca_driver driver.launch"
