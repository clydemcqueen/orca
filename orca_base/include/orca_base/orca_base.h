#ifndef ORCA_BASE_H
#define ORCA_BASE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include "orca_msgs/Depth.h"

namespace orca_base {

enum class Mode
{
  disarmed,       // Thrusters are off; autopilot is off; all joystick buttons except "arm" are ignored
  manual,         // Thrusters are on; manual thruster control
  stabilize,      // Thrusters are on; autopilot is on and controlling yaw
  depth_hold      // Thrusters are on; autopilot is on and controlling yaw and depth
};

// OrcaBase provides basic ROV and AUV functions, including joystick operation, attitude hold, depth hold, and waypoint navigation.
class OrcaBase
{
private:
  ros::NodeHandle &nh_;
  tf::TransformListener &tf_;

  Mode mode_;

  // Yaw pid control state
  double yaw_state_;
  double yaw_setpoint_;
  bool yaw_trim_button_previous_;

  // Depth pid control state
  double depth_state_;
  double depth_setpoint_;
  bool depth_trim_button_previous_;
  
  // Thruster effort from joystick or pid controllers (yaw and depth), ranges from 1.0 for forward to -1.0 for reverse
  double forward_effort_;
  double yaw_effort_;
  double strafe_effort_;
  double vertical_effort_;

  // Camera tilt
  double tilt_;
  bool tilt_trim_button_previous_;

  // Lights
  double lights_;
  bool lights_trim_button_previous_;
  
  // Subscriptions
  ros::Subscriber baro_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber yaw_control_effort_sub_;
  ros::Subscriber depth_control_effort_sub_;
  ros::Subscriber joy_sub_;
  
  // Callbacks
  void baroCallback(const orca_msgs::Depth::ConstPtr &msg);
  void imuCallback(const sensor_msgs::ImuConstPtr &msg);
  void yawControlEffortCallback(const std_msgs::Float64::ConstPtr& msg);
  void depthControlEffortCallback(const std_msgs::Float64::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  
  // Publications
  ros::Publisher thruster_pub_;
  ros::Publisher yaw_pid_enable_pub_;
  ros::Publisher yaw_state_pub_;
  ros::Publisher yaw_setpoint_pub_;
  ros::Publisher depth_pid_enable_pub_;
  ros::Publisher depth_state_pub_;
  ros::Publisher depth_setpoint_pub_;
  ros::Publisher camera_tilt_pub_;
  ros::Publisher lights_pub_;
  
  // Helpers
  void publishYawSetpoint();
  void publishDepthSetpoint();
  void publishCameraTilt();
  void publishLights();
  void setMode(Mode mode, double depth_setpoint);
  
public:
  explicit OrcaBase(ros::NodeHandle &nh, tf::TransformListener &tf);
  ~OrcaBase() {}; // Suppress default copy and move constructors

  void SpinOnce(const ros::TimerEvent &event);
};

} // namespace orca_base

#endif // ORCA_BASE_H