#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include "orca_msgs/Camera.h"

extern "C"
{
  #include <rc_usefulincludes.h>
  #include <roboticscape.h>
}

namespace orca_driver {

// OrcaDriver provides the interface between the BeagleBone Blue hardware and the rest of the Orca stack.
class OrcaDriver
{
private:
  ros::NodeHandle &nh_;
  tf::TransformListener &tf_;
  
  int loop_counter_;
  bool led_on_;
  
  rc_imu_data_t imu_buffer_;
  sensor_msgs::Imu imu_msg_;
  
  // Subscriptions
  ros::Subscriber camera_tilt_sub_;
  
  // Callbacks
  void cameraTiltCallback(const orca_msgs::Camera::ConstPtr &msg);

  // Publications
  ros::Publisher barometer_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher voltage_pub_;
  
  void spinOnce();
  void heartbeat();
  void publishBarometer();
  void publishVoltage();
  
public:
  explicit OrcaDriver(ros::NodeHandle &nh, tf::TransformListener &tf);
  ~OrcaDriver() {}; // Suppress default copy and move constructors
  void publishIMU();
  int run();
};

} // namespace orca_driver

#endif // ORCA_DRIVER_H
