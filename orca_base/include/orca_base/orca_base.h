#ifndef ORCA_BASE_H
#define ORCA_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "orca_msgs/Barometer.h"
#include "orca_msgs/Battery.h"
#include "orca_msgs/Control.h"
#include "orca_msgs/Leak.h"
#include "orca_base/orca_model.h"
#include "orca_base/pid.h"
#include "orca_base/orca_mission.h"

namespace orca_base {

constexpr const bool headingHoldMode(uint8_t mode) { return mode == orca_msgs::Control::hold_h || mode == orca_msgs::Control::hold_hd; };
constexpr const bool depthHoldMode(uint8_t mode) { return mode == orca_msgs::Control::hold_d || mode == orca_msgs::Control::hold_hd; };
constexpr const bool rovMode(uint8_t mode) { return mode == orca_msgs::Control::manual || mode == orca_msgs::Control::hold_h || mode == orca_msgs::Control::hold_d || mode == orca_msgs::Control::hold_hd; }
constexpr const bool auvMode(uint8_t mode) { return mode == orca_msgs::Control::mission; }

// OrcaBase provides basic ROV and AUV functions, including joystick operation, heading hold, depth hold, and waypoint navigation.
class OrcaBase
{
private:
  ros::NodeHandle &nh_;
  ros::NodeHandle &nh_priv_;
  tf2_ros::TransformListener &tf_;

  // Parameters from the parameter server
  int joy_axis_yaw_;
  int joy_axis_forward_;
  int joy_axis_strafe_;
  int joy_axis_vertical_;
  int joy_axis_yaw_trim_;
  int joy_axis_vertical_trim_;
  int joy_button_disarm_;
  int joy_button_arm_;
  int joy_button_manual_;
  int joy_button_hold_h_;
  int joy_button_hold_d_;
  int joy_button_hold_hd_;
  int joy_button_tilt_down_;
  int joy_button_tilt_up_;
  int joy_button_bright_;
  int joy_button_dim_;
  double inc_yaw_;
  double inc_depth_;
  int inc_tilt_;
  int inc_lights_;
  float input_dead_band_;
  double yaw_pid_dead_band_;
  double depth_pid_dead_band_;
  tf2::Matrix3x3 imu_rotation_;

  // General state
  bool simulation_;                   // True if we're in a simulation
  ros::Time ping_time_;               // Last time we heard from the topside
  ros::Time prev_loop_time_;          // Last time spinOnce was called
  uint8_t mode_;                      // Operating mode

  // State estimation and planning
  OrcaOdometry odom_plan_;            // Planned state
  OrcaPose odom_local_;               // Estimated state
  OrcaPose odom_ground_truth_;        // Ground truth

  // Current mission
  std::unique_ptr<BaseMission> mission_;      // The mission we're running
  nav_msgs::Path mission_plan_path_;          // The planned path (data from odom_plan_)
  nav_msgs::Path mission_estimated_path_;     // Best estimate of the actual path (data from odom_local_)
  nav_msgs::Path mission_ground_truth_path_;  // Ground truth (data from odom_ground_truth_)

  // Sensor status
  bool barometer_ready_;              // True if we're receiving barometer messages
  bool gps_ready_;                    // True if we're receiving GPS messages
  bool ground_truth_ready_;           // True if we're receiving ground truth messages
  bool imu_ready_;                    // True if we're receiving IMU messages

  // IMU
  tf2::Matrix3x3 base_orientation_;   // Orientation
  double stability_;                  // Roll and pitch stability from 1.0 (flat) to 0.0 (90 tilt or worse)

  // Yaw controller
  pid::Controller yaw_controller_;
  double yaw_state_;
  double yaw_setpoint_;
  bool yaw_trim_button_previous_;

  // Depth controller
  pid::Controller depth_controller_;
  double depth_adjustment_;
  double depth_state_;
  double depth_setpoint_;
  bool depth_trim_button_previous_;

  // Joystick gain (attenuation), range 0.0 (ignore joystick) to 1.0 (no attenuation)
  double xy_gain_;
  double yaw_gain_;
  double vertical_gain_;

  // Thruster effort from joystick or pid controllers (yaw and depth), ranges from 1.0 for forward to -1.0 for reverse
  OrcaEfforts efforts_;

  // Camera tilt
  int tilt_;
  bool tilt_trim_button_previous_;

  // Lights
  int brightness_;
  bool brightness_trim_button_previous_;

  // Subscriptions
  ros::Subscriber baro_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber ground_truth_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber leak_sub_;
  ros::Subscriber odom_local_sub_;
  ros::Subscriber ping_sub_;

  // Callbacks
  void baroCallback(const orca_msgs::Barometer::ConstPtr &msg);
  void batteryCallback(const orca_msgs::Battery::ConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void gpsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void leakCallback(const orca_msgs::Leak::ConstPtr &msg);
  void odomLocalCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void pingCallback(const std_msgs::Empty::ConstPtr &msg);
  
  // Publications
  ros::Publisher control_pub_;                    // Thruster control messages
  ros::Publisher odom_plan_pub_;                  // Odometry messages
  ros::Publisher thrust_marker_pub_;              // Visualize thrust in rviz
  ros::Publisher mission_plan_pub_;               // Visualize planned path in rviz
  ros::Publisher mission_estimated_pub_;          // Visualize estimated path in rviz
  ros::Publisher mission_ground_truth_pub_;       // Visualize ground truth in rviz

  // Helpers
  void publishControl();
  void publishOdom();
  void setMode(uint8_t new_mode);
  bool holdingHeading() { return headingHoldMode(mode_); };
  bool holdingDepth() { return depthHoldMode(mode_); };
  bool rovOperation() { return rovMode(mode_); };
  bool auvOperation() { return auvMode(mode_); };

public:
  explicit OrcaBase(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, tf2_ros::TransformListener &tf);
  ~OrcaBase() {}; // Suppress default copy and move constructors

  void spinOnce();
};

} // namespace orca_base

#endif // ORCA_BASE_H
