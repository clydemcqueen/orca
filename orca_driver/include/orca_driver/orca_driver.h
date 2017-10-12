#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

extern "C"
{
  #include <rc_usefulincludes.h>
  #include <roboticscape.h>
}

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
  
  int loop_counter_;
  bool led_on_;
  
  rc_imu_data_t imu_buffer_;
  sensor_msgs::Imu imu_msg_;
  
  ros::Publisher barometer_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher voltage_pub_;
  
  void SpinOnce(const ros::TimerEvent &event);
  void Heartbeat();
  void PublishBarometer();
  void PublishVoltage();
  
public:
  explicit OrcaDriver(ros::NodeHandle &nh, tf::TransformListener &tf);
  ~OrcaDriver() {}; // Suppress default copy and move constructors
  void PublishIMU();
  int Run();
};

} // namespace orca_driver

#endif // ORCA_DRIVER_H