#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "orca_base/orca_base.h"
#include "orca_base/orca_pwm.h"

namespace orca_base {

// Limits
constexpr double DEPTH_HOLD_MIN = 0.05; // Hover just below the surface of the water
constexpr double DEPTH_HOLD_MAX = 50;   // Max depth is 100m, but provide a margin of safety

// Message publish rate in Hz
constexpr int SPIN_RATE = 10;

// Timeouts
constexpr double COMM_ERROR_TIMEOUT_DISARM = 5; // Disarm if we can't communicate with the topside
constexpr double COMM_ERROR_TIMEOUT_SOS = 100;  // Panic if it's been too long

struct Thruster
{
  std::string frame_id;   // URDF link frame id
  bool ccw;               // True if counterclockwise
  double forward_factor;
  double strafe_factor;
  double yaw_factor;
  double vertical_factor;
};

// Order must match the order of the <thruster> tags in the URDF
const std::vector<Thruster> THRUSTERS = {
  {"t200_link_front_right", false, 1.0, 1.0, 1.0, 0.0},
  {"t200_link_front_left", false, 1.0, -1.0, -1.0, 0.0},
  {"t200_link_rear_right", true, 1.0, -1.0, 1.0, 0.0},
  {"t200_link_rear_left", true, 1.0, 1.0, -1.0, 0.0},
  {"t200_link_vertical_right", false, 0.0, 0.0, 0.0, 1.0},
  {"t200_link_vertical_left", true, 0.0, 0.0, 0.0, -1.0},
};

OrcaBase::OrcaBase(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, tf2_ros::TransformListener &tf):
  nh_{nh},
  nh_priv_{nh_priv},
  tf_{tf},
  mode_{orca_msgs::Control::disarmed},
  imu_ready_{false},
  barometer_ready_{false},
  tilt_{0},
  tilt_trim_button_previous_{false},
  brightness_{0},
  brightness_trim_button_previous_{false},
  ping_time_{ros::Time::now()},
  prev_loop_time_{ros::Time::now()},
  depth_controller_{false, 0.1, 0, 0.05},
  yaw_controller_{true, 0.007, 0, 0}
{
  nh_priv_.param("joy_axis_yaw", joy_axis_yaw_, 0);                      // Left stick left/right; 1.0 is left and -1.0 is right
  nh_priv_.param("joy_axis_forward", joy_axis_forward_, 1);              // Left stick up/down; 1.0 is forward and -1.0 is backward
  nh_priv_.param("joy_axis_strafe", joy_axis_strafe_, 3);                // Right stick left/right; 1.0 is left and -1.0 is right
  nh_priv_.param("joy_axis_vertical", joy_axis_vertical_, 4);            // Right stick up/down; 1.0 is ascend and -1.0 is descend
  nh_priv_.param("joy_axis_yaw_trim", joy_axis_yaw_trim_, 6);            // Trim left/right; acts like 2 buttons; 1.0 for left and -1.0 for right
  nh_priv_.param("joy_axis_vertical_trim", joy_axis_vertical_trim_, 7);  // Trim up/down; acts like 2 buttons; 1.0 for up and -1.0 for down

  nh_priv_.param("joy_button_disarm", joy_button_disarm_, 6);            // View
  nh_priv_.param("joy_button_arm", joy_button_arm_, 7);                  // Menu
  nh_priv_.param("joy_button_manual", joy_button_manual_, 0);            // A
  nh_priv_.param("joy_button_hold_y", joy_button_hold_h_, 2);            // X
  nh_priv_.param("joy_button_hold_d", joy_button_hold_d_, 1);            // B
  nh_priv_.param("joy_button_hold_hd", joy_button_hold_hd_, 3);          // Y
  nh_priv_.param("joy_button_tilt_down", joy_button_tilt_down_, 4);      // Left bumper
  nh_priv_.param("joy_button_tilt_up", joy_button_tilt_up_, 5);          // Right bumper
  nh_priv_.param("joy_button_bright", joy_button_bright_, 9);            // Left stick
  nh_priv_.param("joy_button_dim", joy_button_dim_, 10);                 // Right stick

  nh_priv_.param("inc_yaw", inc_yaw_, M_PI/36);
  nh_priv_.param("inc_depth", inc_depth_, 0.1);
  nh_priv_.param("inc_tilt", inc_tilt_, 5);
  nh_priv_.param("inc_lights", inc_lights_, 20);
  nh_priv_.param("input_dead_band", input_dead_band_, 0.05f);            // Don't respond to tiny joystick movements
  nh_priv_.param("yaw_pid_dead_band", yaw_pid_dead_band_, 0.005);
  nh_priv_.param("depth_pid_dead_band", depth_pid_dead_band_, 0.005);
  nh_priv_.param("xy_gain", xy_gain_, 0.5);
  nh_priv_.param("yaw_gain", yaw_gain_, 0.2);
  nh_priv_.param("vertical_gain", vertical_gain_, 0.5);

  nh_priv_.param("simulation", simulation_, true);
  if (simulation_)
  {
    ROS_INFO("Running in a simulation");
    imu_rotation_ = tf2::Quaternion::getIdentity();
  }
  else
  {
    ROS_INFO("Running in real life");
    tf2::Quaternion imu_orientation;
    imu_orientation.setRPY(-M_PI/2, -M_PI/2, 0);
    imu_rotation_ =  imu_orientation.inverse();
  }
  ROS_INFO("base_orientation = imu_orientation * {%g, %g, %g, %g}", imu_rotation_.x(), imu_rotation_.y(), imu_rotation_.z(), imu_rotation_.w());

  // Set up all subscriptions
  baro_sub_ = nh_priv_.subscribe<orca_msgs::Barometer>("/barometer", 10, &OrcaBase::baroCallback, this);
  battery_sub_ = nh_priv_.subscribe<orca_msgs::Battery>("/orca_driver/battery", 10, &OrcaBase::batteryCallback, this);
  goal_sub_ = nh_priv_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &OrcaBase::goalCallback, this);
  gps_sub_ = nh_priv_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/gps", 10, &OrcaBase::gpsCallback, this);
  imu_sub_ = nh_priv_.subscribe<sensor_msgs::Imu>("/imu/data", 10, &OrcaBase::imuCallback, this);
  joy_sub_ = nh_priv_.subscribe<sensor_msgs::Joy>("/joy", 10, &OrcaBase::joyCallback, this);
  leak_sub_ = nh_priv_.subscribe<orca_msgs::Leak>("/orca_driver/leak", 10, &OrcaBase::leakCallback, this);
  odom_local_sub_ = nh_priv_.subscribe<nav_msgs::Odometry>("/odometry/local", 10, &OrcaBase::odomLocalCallback, this);
  ping_sub_ = nh_priv_.subscribe<std_msgs::Empty>("/ping", 10, &OrcaBase::pingCallback, this);

  // Advertise all topics that we'll publish on
  control_pub_ = nh_priv_.advertise<orca_msgs::Control>("control", 1);
  odom_plan_pub_ = nh_priv_.advertise<nav_msgs::Odometry>("/odometry/plan", 1);
  thrust_marker_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray>("thrust_markers", 1);
  mission_plan_pub_ = nh_priv_.advertise<nav_msgs::Path>("mission_plan", 1);
  mission_actual_pub_ = nh_priv_.advertise<nav_msgs::Path>("mission_actual", 1);
}

// New barometer reading
void OrcaBase::baroCallback(const orca_msgs::Barometer::ConstPtr& baro_msg)
{
  if (!barometer_ready_)
  {
    // First depth reading: zero the depth
    depth_adjustment_ = baro_msg->depth;
    depth_state_ = 0;
    barometer_ready_ = true;
    ROS_INFO("Barometer ready, depth adjustment %g", depth_adjustment_);
  }
  else
  {
    depth_state_ = baro_msg->depth - depth_adjustment_;
  }
}

// New battery reading
void OrcaBase::batteryCallback(const orca_msgs::Battery::ConstPtr& battery_msg)
{
  if (battery_msg->low_battery)
  {
    ROS_ERROR("SOS! Low battery! %g volts", battery_msg->voltage);
    setMode(orca_msgs::Control::sos);
  }
}

// New 2D goal
void OrcaBase::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (mode_ != orca_msgs::Control::disarmed && barometer_ready_ && gps_ready_ && imu_ready_)
  {
    // Pull out yaw (kinda cumbersome)
    tf2::Quaternion goal_orientation;
    tf2::fromMsg(msg->pose.orientation, goal_orientation);
    double roll, pitch, goal_yaw;
    tf2::Matrix3x3(goal_orientation).getRPY(roll, pitch, goal_yaw);

    OrcaPose goal_pose(msg->pose.position.x, msg->pose.position.y, -depth_state_, goal_yaw);

    ROS_INFO("Start mission at (%g, %g), goal is (%g, %g), heading %g", odom_local_.x, odom_local_.y, goal_pose.x, goal_pose.y, goal_pose.yaw);

    mission_.reset(new SquareMission());
    if (mission_->init(odom_local_, goal_pose))
    {
      mission_plan_path_.header.stamp = ros::Time::now();
      mission_plan_path_.header.frame_id = "map";
      mission_plan_path_.poses.clear();

      mission_estimated_path_.header.stamp = ros::Time::now();
      mission_estimated_path_.header.frame_id = "map";
      mission_estimated_path_.poses.clear();

      // Init plan
      odom_plan_ = odom_local_;

      setMode(orca_msgs::Control::mission);
    }
    else
    {
      ROS_ERROR("Can't initialize mission; internal error");
    }
  }
  else
  {
    ROS_ERROR("Can't start mission; possible reasons: disarmed, barometer not ready, GPS not ready, IMU not ready");
  }
}

// New GPS reading
void OrcaBase::gpsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if (!gps_ready_)
  {
    gps_ready_ = true;
    ROS_INFO("GPS ready (%g, %g, %g)", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }
}

// New IMU reading
void OrcaBase::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  // Compute base_orientation
  tf2::Quaternion imu_orientation;
  tf2::fromMsg(msg->orientation, imu_orientation);
  if (simulation_)
  {
    // Gazebo IMU noise model might result in non-normalized Quaternion
    imu_orientation = imu_orientation.normalize();
  }
  base_orientation_ = imu_orientation * imu_rotation_;

  // Pull out yaw
  double roll, pitch;
  tf2::Matrix3x3(base_orientation_).getRPY(roll, pitch, yaw_state_);

  // Compute a stability metric, used to throttle the pid controllers
  stability_ = std::min(clamp(std::cos(roll), 0.0, 1.0), clamp(std::cos(pitch), 0.0, 1.0));

#if 0
  // NWU to ENU
  yaw_state_ += M_PI_2;
  base_orientation_.setRPY(roll, pitch, yaw_state_);
#endif

  if (!imu_ready_)
  {
    imu_ready_ = true;
    ROS_INFO("IMU ready, roll %4.2f pitch %4.2f yaw %4.2f", roll, pitch, yaw_state_);
  }
}

// Leak detector
void OrcaBase::leakCallback(const orca_msgs::Leak::ConstPtr& leak_msg)
{
  if (leak_msg->leak_detected)
  {
    ROS_ERROR("SOS! Leak detected!");
    setMode(orca_msgs::Control::sos);
  }
}

// Ping from topside
void OrcaBase::pingCallback(const std_msgs::Empty::ConstPtr& msg)
{
  ping_time_ = ros::Time::now();
}

// Local odometry -- result from robot_localization, fusing all continuous sensors
void OrcaBase::odomLocalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_local_.fromMsg(*msg);
}

void OrcaBase::publishOdom()
{
  // TODO pull everything (including time) from motion planner; this is effectively the "keep station" planner

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";

  odom_msg.pose.pose.position.x = 0;
  odom_msg.pose.pose.position.y = 0;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = geometry_msgs::Quaternion();

  odom_msg.twist.twist.linear = geometry_msgs::Vector3();
  odom_msg.twist.twist.angular = geometry_msgs::Vector3();

  for (int i = 0; i < 6; ++i)
  {
    odom_msg.pose.covariance[i * 6 + i] = 0.01;
    odom_msg.twist.covariance[i * 6 + i] = 0.01;
  }

  odom_plan_pub_.publish(odom_msg);
}

void OrcaBase::publishControl()
{
  // Combine joystick efforts to get thruster efforts.
  std::vector<double> thruster_efforts = {};
  for (int i = 0; i < THRUSTERS.size(); ++i)
  {
    // Clamp forward + strafe to xy_gain_
    double xy_effort = clamp(efforts_.forward * THRUSTERS[i].forward_factor + efforts_.strafe * THRUSTERS[i].strafe_factor,
      -xy_gain_, xy_gain_);

    // Clamp total thrust
    thruster_efforts.push_back(clamp(xy_effort + efforts_.yaw * THRUSTERS[i].yaw_factor + efforts_.vertical * THRUSTERS[i].vertical_factor,
      THRUST_FULL_REV, THRUST_FULL_FWD));
  }

  // Publish control message
  orca_msgs::Control control_msg;
  control_msg.header.stamp = ros::Time::now();
  control_msg.mode = mode_;
  control_msg.camera_tilt_pwm = tilt_to_pwm(tilt_);
  control_msg.brightness_pwm = brightness_to_pwm(brightness_);
  for (int i = 0; i < thruster_efforts.size(); ++i)
  {
    control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_efforts[i]));
  }
  control_pub_.publish(control_msg);

  // Publish rviz marker message
  visualization_msgs::MarkerArray markers_msg;
  for (int i = 0; i < thruster_efforts.size(); ++i)
  {
    int32_t action = thruster_efforts[i] == 0.0 ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
    double scale = (THRUSTERS[i].ccw ? -thruster_efforts[i] : thruster_efforts[i]) / 5.0;
    double offset = scale > 0 ? -0.1 : 0.1;

    visualization_msgs::Marker marker;
    marker.header.frame_id = THRUSTERS[i].frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "thruster";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = action;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = offset;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.7071068;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.7071068;
    marker.scale.x = scale;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markers_msg.markers.push_back(marker);
  }
  thrust_marker_pub_.publish(markers_msg);
}

// Change operation mode
void OrcaBase::setMode(uint8_t new_mode)
{
  // Stop all thrusters when we change modes
  efforts_.clear();

  if (auvOperation() && rovMode(new_mode))
  {
    // AUV to ROV transition, start the communication clock
    ping_time_ = ros::Time::now();
  }

  if (depthHoldMode(new_mode)) // TODO move to keep station planner
  {
    // Set target depth
    depth_setpoint_ = depth_state_;
    depth_controller_.setTarget(depth_setpoint_);

    // Clear button state
    depth_trim_button_previous_ = false;
  }

  if (headingHoldMode(new_mode)) // TODO move to keep station planner
  {
    // Set target angle
    yaw_setpoint_ = yaw_state_;
    yaw_controller_.setTarget(yaw_setpoint_);

    // Clear button state
    yaw_trim_button_previous_ = false;
  }

  if (new_mode == orca_msgs::Control::disarmed)
  {
    // Turn off lights
    brightness_ = 0;
  }

  // TODO sos

  // Set the new mode
  mode_ = new_mode;
}

// New input from the gamepad
void OrcaBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // A joystick message means that we're talking to the topside
  ping_time_ = ros::Time::now();

  // If we're in trouble, ignore the joystick
  if (mode_ == orca_msgs::Control::sos)
  {
    ROS_INFO("SOS, ignoring joystick");
    return;
  }

  // Arm/disarm
  if (joy_msg->buttons[joy_button_disarm_])
  {
    ROS_INFO("Disarmed");
    setMode(orca_msgs::Control::disarmed);
  }
  else if (joy_msg->buttons[joy_button_arm_])
  {
    ROS_INFO("Armed, manual");
    setMode(orca_msgs::Control::manual);
  }

  // If we're disarmed, ignore everything else
  if (mode_ == orca_msgs::Control::disarmed)
  {
    ROS_INFO("Disarmed, ignoring further input");
    return;
  }

  // Mode
  if (joy_msg->buttons[joy_button_manual_])
  {
    ROS_INFO("Manual");
    setMode(orca_msgs::Control::manual);
  }
  else if (joy_msg->buttons[joy_button_hold_h_])
  {
    if (imu_ready_)
    {
      ROS_INFO("Hold heading");
      setMode(orca_msgs::Control::hold_h);
    }
    else
    {
      ROS_ERROR("IMU not ready, can't hold heading");
    }
  }
  else if (joy_msg->buttons[joy_button_hold_d_])
  {
    if (barometer_ready_)
    {
      ROS_INFO("Hold depth");
      setMode(orca_msgs::Control::hold_d);
    }
    else
    {
      ROS_ERROR("Barometer not ready, can't hold depth");
    }
  }
  else if (joy_msg->buttons[joy_button_hold_hd_])
  {
    if (imu_ready_ && barometer_ready_)
    {
      ROS_INFO("Hold heading and depth");
      setMode(orca_msgs::Control::hold_hd);
    }
    else
    {
      ROS_ERROR("Barometer and/or IMU not ready, can't hold heading and depth");
    }
  }

  // Yaw trim
  if (joy_msg->axes[joy_axis_yaw_trim_] != 0.0 && !yaw_trim_button_previous_)
  {
    // Rising edge
    if ((holdingHeading()))
    {
      yaw_setpoint_ = joy_msg->axes[joy_axis_yaw_trim_] > 0.0 ? yaw_setpoint_ + inc_yaw_ : yaw_setpoint_ - inc_yaw_;
      yaw_controller_.setTarget(yaw_setpoint_);
    }

    yaw_trim_button_previous_ = true;
  }
  else if (joy_msg->axes[joy_axis_yaw_trim_] == 0.0 && yaw_trim_button_previous_)
  {
    // Falling edge
    yaw_trim_button_previous_ = false;
  }

  // Depth trim
  if (joy_msg->axes[joy_axis_vertical_trim_] != 0.0 && !depth_trim_button_previous_)
  {
    // Rising edge
    if (holdingDepth())
    {
      depth_setpoint_ = clamp(joy_msg->axes[joy_axis_vertical_trim_] < 0 ? depth_setpoint_ + inc_depth_ : depth_setpoint_ - inc_depth_,
        DEPTH_HOLD_MIN, DEPTH_HOLD_MAX);
      depth_controller_.setTarget(depth_setpoint_);
    }

    depth_trim_button_previous_ = true;
  }
  else if (joy_msg->axes[joy_axis_vertical_trim_] == 0.0 && depth_trim_button_previous_)
  {
    // Falling edge
    depth_trim_button_previous_ = false;
  }

  // Camera tilt
  if ((joy_msg->buttons[joy_button_tilt_up_] || joy_msg->buttons[joy_button_tilt_down_]) && !tilt_trim_button_previous_)
  {
    // Rising edge
    tilt_ = clamp(joy_msg->buttons[joy_button_tilt_up_] ? tilt_ + inc_tilt_ : tilt_ - inc_tilt_, TILT_MIN, TILT_MAX);
    tilt_trim_button_previous_ = true;
  }
  else if (!joy_msg->buttons[joy_button_tilt_up_] && !joy_msg->buttons[joy_button_tilt_down_] && tilt_trim_button_previous_)
  {
    // Falling edge
    tilt_trim_button_previous_ = false;
  }

  // Lights
  if ((joy_msg->buttons[joy_button_bright_] || joy_msg->buttons[joy_button_dim_]) && !brightness_trim_button_previous_)
  {
    // Rising edge
    brightness_ = clamp(joy_msg->buttons[joy_button_bright_] ? brightness_ + inc_lights_ : brightness_ - inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    brightness_trim_button_previous_ = true;
  }
  else if (!joy_msg->buttons[joy_button_bright_] && !joy_msg->buttons[joy_button_dim_] && brightness_trim_button_previous_)
  {
    // Falling edge
    brightness_trim_button_previous_ = false;
  }

  // Thrusters
  if (rovOperation())
  {
    efforts_.forward = dead_band(joy_msg->axes[joy_axis_forward_], input_dead_band_) * xy_gain_;
    if (!holdingHeading())
    {
      efforts_.yaw = dead_band(joy_msg->axes[joy_axis_yaw_], input_dead_band_) * yaw_gain_;
    }
    efforts_.strafe = dead_band(joy_msg->axes[joy_axis_strafe_], input_dead_band_) * xy_gain_;
    if (!holdingDepth())
    {
      efforts_.vertical = dead_band(joy_msg->axes[joy_axis_vertical_], input_dead_band_) * vertical_gain_;
    }
  }
}

// Our main loop
void OrcaBase::spinOnce()
{
  ros::Time now = ros::Time::now();
  double dt = (now - prev_loop_time_).toSec();
  prev_loop_time_ = now;

  // Check for communication problems
  if (rovOperation() && now - ping_time_ > ros::Duration(COMM_ERROR_TIMEOUT_DISARM))
  {
    if (now - ping_time_ > ros::Duration(COMM_ERROR_TIMEOUT_SOS))
    {
      ROS_ERROR("SOS! Lost contact for way too long");
      setMode(orca_msgs::Control::sos);
    }
    else
    {
      ROS_ERROR("Lost contact with topside; disarming");
      setMode(orca_msgs::Control::disarmed);
    }
  }

  // Compute yaw effort
  if (holdingHeading())
  {
    double effort = yaw_controller_.calc(yaw_state_, dt, 0);
    efforts_.yaw = dead_band(effort * stability_, yaw_pid_dead_band_);
  }

  // Compute depth effort
  if (holdingDepth())
  {
    double effort = depth_controller_.calc(depth_state_, dt, 0);
    efforts_.vertical = dead_band(-effort * stability_, depth_pid_dead_band_);
  }

  // Run a mission
  if (mode_ == orca_msgs::Control::mission)
  {
    BaseMission::addToPath(mission_estimated_path_, odom_local_);
    mission_actual_pub_.publish(mission_estimated_path_);

    if (mission_->advance(odom_local_, odom_plan_, efforts_))
    {
      BaseMission::addToPath(mission_plan_path_, odom_plan_);
      mission_plan_pub_.publish(mission_plan_path_);

      // TODO deadband?
      efforts_.forward = clamp(efforts_.forward * stability_, -1.0, 1.0);
      efforts_.strafe = clamp(efforts_.strafe * stability_, -1.0, 1.0);
      efforts_.vertical = clamp(-efforts_.vertical * stability_, -1.0, 1.0);
      efforts_.yaw = clamp(efforts_.yaw * stability_, -1.0, 1.0);
    }
    else
    {
      ROS_INFO("Mission complete");
      setMode(orca_msgs::Control::manual);
    }
  }

  // Publish controls for thrusters, lights and camera tilt
  publishControl();

  // Publish odometry
  publishOdom();
}

} // namespace orca_base

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orca_base");
  ros::NodeHandle nh{""};
  ros::NodeHandle nh_priv{"~"};
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf{tf_buffer};
  orca_base::OrcaBase orca_base{nh, nh_priv, tf};

  ROS_INFO("Entering main loop");
  ros::Rate r(orca_base::SPIN_RATE);
  while (ros::ok())
  {
    // Do our work
    orca_base.spinOnce();

    // Respond to incoming messages
    ros::spinOnce();

    // Wait
    r.sleep();
  }

  return 0;
}
