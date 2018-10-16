# Orca #

Orca is a [ROS](http://ros.org) driver for [BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/).
Version 0.1.0  supports basic ROV functions; subsequent versions will add AUV functions.

Note: Orca requires hardware modifications to the BlueROV2.
It does not use the Pixhawk controller that comes with the BlueROV2, and does not support [mavros](http://wiki.ros.org/mavros), [ArduSub](https://www.ardusub.com/) or [QGroundControl](http://qgroundcontrol.com/).

## Tested hardware

* [BlueRobotics BlueROV2](https://www.bluerobotics.com/store/rov/bluerov2/), with the included Raspberry Pi 3, Fathom-X and Bar30
* [Pololu Maestro 18-Channel USB Servo Controller](https://www.pololu.com/product/1354)
* [PhidgetSpatial Precision 3/3/3 High Resolution IMU](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=32)
* Xbox One gamepad

## Simulation

Orca runs in [Gazebo](http://gazebosim.org/), a SITL (software-in-the-loop) simulator.
Use the instructions below to install ROS, Gazebo and Orca on your desktop or laptop.

Install [ROS Melodic](http://wiki.ros.org/Installation/Ubuntu).
Select `ros-melodic-desktop-full`; this will install Gazebo 9 as well.

Install these additional packages:
~~~~
sudo apt install ros-melodic-imu-tools ros-melodic-robot-localization
~~~~

Create a catkin workspace:
~~~~
source /opt/ros/melodic/setup.bash
mkdir -p ~/orca_catkin_ws/src
cd ~/orca_catkin_ws/
catkin_make
source devel/setup.bash
~~~~

Download and compile Orca:
~~~~
cd ~/orca_catkin_ws/src
git clone https://github.com/clydemcqueen/orca.git
cd ..
catkin_make
~~~~

Run the simulation:
~~~~
roslaunch orca_gazebo gazebo.launch --screen
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

## Design

I considered 3 ways to ROSify a BlueROV2:

* Use the existing hardware (Pixhawk) and software (ArduSub), and use [mavros](http://wiki.ros.org/mavros) to move messages from the MAV message bus to/from ROS.
See [BlueRov-ROS-playground](https://github.com/patrickelectric/bluerov_ros_playground) for a good example of this method.
This is the fastest way to integrate ROS with the BlueROV2.
* Port ROS to the Pixhawk and NuttX, and run a ROS-native driver on the Pixhawk. I don't know of anybody working on this.
* Provide a ROS-native driver running on Linux, such as the Raspberry Pi 3.
You'll need to provide a small device controller, such as the Pololu Maestro, and an IMU, such as the Phidgets IMU.
This is the Orca design.

There are 7 projects:
* `orca_msgs` provides message types
* `orca_description` provides robot description files
* `orca_driver` provides the interface between the hardware and ROS (not required for simulations)
* `orca_base` provides the ROV functionality
* `orca_topside` provides the topside environment (rviz is used instead of QGroundControl)
* `orca_gazebo` provides the simulation environment
* `orca_vision` provides a stereo vision system (experimental)

## Hardware modifications

This is rough sketch of the hardware modifications I made to the BlueROV2. YMMV.

* Remove the Pixhawk and Pixhawk power supply
* Install the Maestro and connect to Pi3 via USB; set the jumper to isolate the Maestro power rail from USB-provide power
* Install the IMU and connect to the Pi3 via USB
* Connect the Bar30 to the Pi3 I2C and 3.3V power pins
* Connect the ESCs to the Maestro; cut the power wire on all but one ESC
* Connect the camera tilt servo to the Maestro
* Connect the lights signal wire to the Maestro; provide power and ground directly from the battery
* Connect the leak detector to a Maestro digital input
* Build a voltage divider to provide a voltage signal from the battery (0-17V) to a Maestro analog input (0-5V)

Orca power budget:

* The BlueROV2 5V 3A regulated power supply provides power for the RPi (2.5A), the Maestro (50mA), the leak detector (20mA), the Bar30 (2mA), the IMU (55mA) and the RPi camera (250mA)
* The Fathom-X, LED lights and ESCs are powered directly from the battery
* The camera tilt servo is powered from the 5V 500mA regulator on one (just one!) of the ESCs. Set the jumpers on the Maestro to isolate the servo rail from the USB-provided power

Notes on the RPi3 power needs:

* The RPi3 needs a 5V 2.5A power supply. It can provide 1.2A to all USB devices
* The RPi3 can provide 50mA across all GPIO pins, but just 16mA for any particular pin
* Note that the RPi3 pins are 3.3V, not 5V