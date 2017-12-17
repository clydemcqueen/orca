# Orca #

Orca is a [ROS](http://ros.org) driver for [BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/). Version 1.0 will support basic ROV functions; subsequent versions will add AUV functions.

Orca does not use the Pixhawk controller that comes with the BlueROV2, and does not support [mavros](http://wiki.ros.org/mavros), [ArduSub](https://www.ardusub.com/) or [QGroundControl](http://qgroundcontrol.com/).

## Tested Hardware

* [BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/)
* [BlueRobotics Low-Light HD USB Camera](https://www.bluerobotics.com/store/electronics/cam-usb-low-light-r1/)
* [BlueRobotics Bar30 Depth Sensor](https://www.bluerobotics.com/store/electronics/bar30-sensor-r1/)
* [Pololu Maestro controller](https://www.pololu.com/product/1354)
* [PhidgetSpatial Precision 3/3/3 High Resolution IMU](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=32)
* Raspberry Pi 3
* Xbox One gamepad

## Install

*Current status: a very early version of Orca runs in [Gazebo](http://gazebosim.org/), a SITL (software-in-the-loop) simulator. Use the instructions below to install ROS, Gazebo and Orca on your desktop or laptop.*

Install [ROS Kinetic](http://wiki.ros.org/Installation/Ubuntu). Select `ros-kinetic-desktop-full`; this will install Gazebo 7.0 as well.

Install these additional packages:
~~~~
sudo apt install ros-kinetic-pid libqt5gstreamer-dev
~~~~

Create a catkin workspace:
~~~~
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
~~~~

Download and compile Orca:
~~~~
cd ~/catkin_ws/src
git clone https://github.com/clydemcqueen/orca.git
cd ..
catkin_make
~~~~

Run the simulation:
~~~~
roslaunch --screen orca_gazebo orca.launch
~~~~

Plug in your gamepad, hit the [menu button](https://support.xbox.com/en-US/xbox-one/accessories/xbox-one-wireless-controller) to arm the thrusters and start driving around. Here's how the buttons are mapped:
* Left stick up/down is forward/reverse
* Left stick left/right is yaw left/right
* Right stick up/down is ascend/descend
* Right stick left/right is strafe left/right
* Menu button: arm
* View button: disarm
* A button: manual
* X button: hold heading
* B button: hold depth
* Y button: hold heading and depth

## Code Structure

There are 6 projects:
* `orca_msgs` provides message types
* `orca_description` provides robot description files (urdf, etc.)
* `orca_driver` provides the interface between the hardware and ROS (not required for simulations)
* `orca_base` provides the ROV and AUV functionality
* `orca_topside` provides the topside environment, including rviz configuration
* `orca_gazebo` provides the simulation environment