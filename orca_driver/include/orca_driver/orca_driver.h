#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <vector>
#include <ros/ros.h>
#include "orca_driver/maestro.h"
#include "orca_msgs/Battery.h"
#include "orca_msgs/Camera.h"
#include "orca_msgs/Leak.h"
#include "orca_msgs/Lights.h"
#include "orca_msgs/Thrusters.h"

template<class T>
constexpr const T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}
  
constexpr const int servo_pulse_width(const float v, const float v_min, const float v_max, const unsigned int pwm_min, const unsigned int pwm_max)
{
  return clamp(1100 + static_cast<unsigned int>(800.0 / (v_max - v_min) * (v - v_min)), pwm_min, pwm_max);
}

namespace orca_driver {

// OrcaDriver provides the interface between the Orca hardware and ROS.

class OrcaDriver
{
private:
  ros::NodeHandle &nh_;
  std::vector<int> thruster_channels_;
  maestro::Maestro maestro_;
  orca_msgs::Battery battery_msg_;
  orca_msgs::Leak leak_msg_;
  
  // Subscriptions
  ros::Subscriber camera_tilt_sub_;
  ros::Subscriber lights_sub_;
  ros::Subscriber thruster_sub_;
  
  // Callbacks
  void cameraTiltCallback(const orca_msgs::Camera::ConstPtr &msg);
  void lightsCallback(const orca_msgs::Lights::ConstPtr &msg);
  void thrustersCallback(const orca_msgs::Thrusters::ConstPtr &msg);
  
  // Publications
  ros::Publisher battery_pub_;
  ros::Publisher leak_pub_;
  
  void spinOnce();
  bool readBattery();
  bool readLeak();
  bool preDive();

public:
  explicit OrcaDriver(ros::NodeHandle &nh);
  ~OrcaDriver() {}; // Suppress default copy and move constructors

  bool run();
};

} // namespace orca_driver

#endif // ORCA_DRIVER_H
