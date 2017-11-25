#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include "orca_driver/maestro.h"
#include "orca_msgs/Battery.h"
#include "orca_msgs/Camera.h"
#include "orca_msgs/Leak.h"
#include "orca_msgs/Lights.h"
#include "orca_msgs/Thrusters.h"

namespace orca_driver {

struct Thruster
{
  int channel_;
  bool reverse_;
};

// OrcaDriver provides the interface between the Orca hardware and ROS.

class OrcaDriver
{
private:
  ros::NodeHandle &nh_;
  ros::NodeHandle &nh_priv_;

  // Parameters
  int num_thrusters_;
  std::vector<Thruster> thrusters_;
  double thruster_limit_;
  int lights_channel_;
  int tilt_channel_;
  int voltage_channel_;
  int leak_channel_;
  int spin_rate_;
  std::string maestro_port_;
  double voltage_multiplier_;
  double voltage_min_;

  // State
  maestro::Maestro maestro_;
  orca_msgs::Battery battery_msg_;
  orca_msgs::Leak leak_msg_;
  
  // Subscriptions
  ros::Subscriber camera_tilt_sub_;
  ros::Subscriber lights_sub_;
  ros::Subscriber thrusters_sub_;
  
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
  explicit OrcaDriver(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
  ~OrcaDriver() {}; // Suppress default copy and move constructors

  bool run();
};

} // namespace orca_driver

#endif // ORCA_DRIVER_H
