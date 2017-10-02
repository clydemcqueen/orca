#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <ros/ros.h>

#include <tf/transform_listener.h>

// OrcaDriver provides the interface between the BeagleBone Blue hardware and the rest of the Orca stack. OrcaDriver can be simulated in Gazebo.
//
// ROS messages we listen to:
// -- TODO thruster
// -- TODO camera tilt
// -- TODO lights
//
// ROS messages we publish:
// -- TODO odom ??
// -- TODO baro
// -- TODO imu
// -- TODO camera

namespace orca_driver {

class OrcaDriver
{
private:
  ros::NodeHandle &nh_;
  tf::TransformListener &tf_;  
  
  // TODO thruster

  // TODO camera tilt

  // TODO lights

  // TODO odom

  // TODO baro

  // TODO imu

  // TODO camera

public:
  explicit OrcaDriver(ros::NodeHandle &nh, tf::TransformListener &tf);
  ~OrcaDriver() {}; // Suppress default copy and move constructors

  void SpinOnce(const ros::TimerEvent &event);
};

} // namespace orca_driver

#endif // ORCA_DRIVER_H