## Orca stereo vision

This is a work in progress. YMMV.

Goals:
* use a forward-facing stereo camera system to avoid obstacles
* use a down-facing stereo camera system to avoid obstacles, determine altitude and map the seafloor

Stereo vision requires 2 identical cameras mounted on the ROV frame streaming h264 video.
The video is processed in the main electronics tube running ROS, gstreamer and OpenCV3.

Example camera:
* BlueRobotics 2" series tube with dome
* Raspberry Pi camera with wide angle lens
* Raspberry Pi Zero running gst-launch-1.0, streaming h264 to the main tube
* USB to Ethernet adapter with passive PoE
* UBEC to power the Pi Zero and camera from the main battery
* Misc hardware

Main tube modifications:
* Add an Ethernet switch
* Add a video processor capable of running ROS stereo vision algorithms, such as a 4-core Atom x86 board from up-board.org. 

All devices require gstreamer-1.0:
~~~~
sudo apt install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools
~~~~

The video processor requires ROS Kinetic, Orca and the latest gscam driver:
~~~~
cd ~/orca_catkin_ws/src
git clone https://github.com/ros-drivers/gscam.git
~~~~

Launch:
~~~~
roslaunch orca_vision stereo.launch
~~~~