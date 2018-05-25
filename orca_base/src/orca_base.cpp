#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include "orca_base/orca_base.h"
#include "orca_base/orca_pwm.h"

namespace orca_base {

// Limits
constexpr double DEPTH_HOLD_MIN = 0.05; // Hover just below the surface of the water
constexpr double DEPTH_HOLD_MAX = 50;   // Max depth is 100m, but provide a margin of safety

// Message publish rate in Hz
constexpr int SPIN_RATE = 10;

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
  forward_effort_{0},
  yaw_effort_{0},
  strafe_effort_{0},
  vertical_effort_{0},
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
    imu_rotation_ = tf2::Quaternion::getIdentity();
  }
  else
  {
    tf2::Quaternion imu_orientation;
    imu_orientation.setRPY(-M_PI/2, -M_PI/2, 0);
    imu_rotation_ =  imu_orientation.inverse();
  }

  // Set up all subscriptions
  baro_sub_ = nh_priv_.subscribe<orca_msgs::Barometer>("/barometer", 10, &OrcaBase::baroCallback, this);
  imu_sub_ = nh_priv_.subscribe<sensor_msgs::Imu>("/imu/data", 10, &OrcaBase::imuCallback, this);
  joy_sub_ = nh_priv_.subscribe<sensor_msgs::Joy>("/joy", 10, &OrcaBase::joyCallback, this);
  ping_sub_ = nh_priv_.subscribe<std_msgs::Empty>("/ping", 10, &OrcaBase::pingCallback, this);

  // Advertise all topics that we'll publish on
  control_pub_ = nh_priv_.advertise<orca_msgs::Control>("control", 1);
  marker_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray>("rviz_marker_array", 1);
  odom_pub_ = nh_priv_.advertise<nav_msgs::Odometry>("/odom", 1);
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

// New imu reading
void OrcaBase::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  // IMU orientation, rotated to account for the placement in the ROV
  tf2::Quaternion imu_orientation;
  tf2::fromMsg(msg->orientation, imu_orientation);
  if (simulation_)
  {
    // Gazebo IMU noise model might result in non-normalized Quaternion
    imu_orientation = imu_orientation.normalize();
  }
  base_orientation_ = imu_orientation * imu_rotation_;

  double roll, pitch;
  tf2::Matrix3x3(base_orientation_).getRPY(roll, pitch, yaw_state_);

  // NWU to ENU
  yaw_state_ += M_PI_2;
  base_orientation_.setRPY(roll, pitch, yaw_state_);

  // Compute a stability metric, used to throttle the pid controllers
  stability_ = std::min(clamp(std::cos(roll), 0.0, 1.0), clamp(std::cos(pitch), 0.0, 1.0));

  // Estimate position -- experimental
  if (imu_ready_)
  {
    tf2::Vector3 linear_acceleration;
    tf2::fromMsg(msg->linear_acceleration, linear_acceleration);
    double west = linear_acceleration.y();
    linear_acceleration.setY(linear_acceleration.x()); // NWU to ENU
    linear_acceleration.setX(-west); // NWU to ENU
    tf2::fromMsg(msg->angular_velocity, angular_velocity_);// TODO NWU to ENU?
    // TODO get covariance
    double delta = (msg->header.stamp - imu_msg_time_).toSec();
    linear_velocity_ += linear_acceleration * delta;
    position_ += linear_velocity_ * delta;
  }
  imu_msg_time_ = msg->header.stamp;

  if (!imu_ready_)
  {
    imu_ready_ = true;
    ROS_INFO("IMU ready, roll %4.2f pitch %4.2f yaw %4.2f", roll, pitch, yaw_state_);
  }
}

// Ping from topside
void OrcaBase::pingCallback(const std_msgs::Empty::ConstPtr& msg)
{
  ping_time_ = ros::Time::now();
}

void OrcaBase::publishOdom()
{
  if (imu_ready_ && barometer_ready_)
  {
    // Publish a transform from base_link to odom
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = imu_msg_time_;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = 0; // position_.x();
    odom_tf.transform.translation.y = 0; // position_.y();
    odom_tf.transform.translation.z = -depth_state_;
    odom_tf.transform.rotation = tf2::toMsg(base_orientation_);
    tf_broadcaster_.sendTransform(odom_tf);

    // Publish an odom message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = imu_msg_time_;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 0; // position_.x();
    odom_msg.pose.pose.position.y = 0; // position_.y();
    odom_msg.pose.pose.position.z = -depth_state_;
    odom_msg.pose.pose.orientation = tf2::toMsg(base_orientation_);
    odom_msg.twist.twist.angular = tf2::toMsg(angular_velocity_);
    odom_msg.twist.twist.linear = tf2::toMsg(linear_velocity_);
    // TODO compute covariance
    odom_pub_.publish(odom_msg);
  }
}

void OrcaBase::publishControl()
{
  // Combine joystick efforts to get thruster efforts.
  std::vector<double> thruster_efforts = {};
  for (int i = 0; i < THRUSTERS.size(); ++i)
  {
    // Clamp forward + strafe to xy_gain_
    double xy_effort = clamp(forward_effort_ * THRUSTERS[i].forward_factor + strafe_effort_ * THRUSTERS[i].strafe_factor,
      -xy_gain_, xy_gain_);

    // Clamp total thrust
    thruster_efforts.push_back(clamp(xy_effort + yaw_effort_ * THRUSTERS[i].yaw_factor + vertical_effort_ * THRUSTERS[i].vertical_factor,
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
  marker_pub_.publish(markers_msg);
}

// Change operation mode
void OrcaBase::setMode(uint8_t mode)
{
  mode_ = mode;  

  if (holdingDepth())
  {
    // Set target depth
    depth_setpoint_ = depth_state_;
    depth_controller_.setTarget(depth_setpoint_);

    // Clear button state
    depth_trim_button_previous_ = false;
  }

  if (holdingHeading())
  {
    // Set target angle
    yaw_setpoint_ = yaw_state_;
    yaw_controller_.setTarget(yaw_setpoint_);

    // Clear button state
    yaw_trim_button_previous_ = false;
  }

  if (mode == orca_msgs::Control::disarmed)
  {
    forward_effort_ = yaw_effort_ = strafe_effort_ = vertical_effort_ = brightness_ = 0.0; // TODO type mismatch
  }
}

// New input from the gamepad
void OrcaBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // A joystick message means that we're talking to the topside
  ping_time_ = ros::Time::now();

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
  forward_effort_ = dead_band(joy_msg->axes[joy_axis_forward_], input_dead_band_) * xy_gain_;
  if (!holdingHeading())
  {
    yaw_effort_ = dead_band(joy_msg->axes[joy_axis_yaw_], input_dead_band_) * yaw_gain_;
  }
  strafe_effort_ = dead_band(joy_msg->axes[joy_axis_strafe_], input_dead_band_) * xy_gain_;
  if (!holdingDepth())
  {
    vertical_effort_ = dead_band(joy_msg->axes[joy_axis_vertical_], input_dead_band_) * vertical_gain_;
  }
}

// Our main loop
void OrcaBase::spinOnce()
{
  ros::Time now = ros::Time::now();
  double dt = (now - prev_loop_time_).toSec();
  prev_loop_time_ = now;

  // If we're not getting messages from the topside, disarm and wait TODO handle ROV vs AUV case
  if (now - ping_time_ > ros::Duration(5.0) && mode_ != orca_msgs::Control::disarmed)
  {
    ROS_ERROR("Lost contact with topside; disarming");
    setMode(orca_msgs::Control::disarmed);
  }

  // Compute yaw effort
  if (holdingHeading())
  {
    double effort = yaw_controller_.calc(yaw_state_, dt, 0);
    yaw_effort_ = dead_band(effort * stability_, yaw_pid_dead_band_);
  }

  // Compute depth effort
  if (holdingDepth())
  {
    double effort = depth_controller_.calc(depth_state_, dt, 0);
    vertical_effort_ = dead_band(-effort * stability_, depth_pid_dead_band_);
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
