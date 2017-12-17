#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "orca_base/orca_base.h"
#include "orca_msgs/Control.h"

// Limits
constexpr double THRUSTER_MIN = -1.0;
constexpr double THRUSTER_MAX = 1.0;
constexpr int TILT_MIN = -45;
constexpr int TILT_MAX = 45;
constexpr int LIGHTS_MIN = 0;
constexpr int LIGHTS_MAX = 100;

// Message publish rate in Hz
constexpr int SPIN_RATE = 10;

template<class T>
constexpr const T dead_band(const T v, const T d)
{
  return v < d && v > -d ? 0 : v;
}

template<class T>
constexpr const T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

namespace orca_base {

struct Thruster
{
  std::string frame_id;   // URDF link frame id
  bool ccw;               // True if counterclockwise
};

const std::vector<Thruster> THRUSTERS = {
  {"t200_link_front_right", false},
  {"t200_link_front_left", false},
  {"t200_link_rear_right", true},
  {"t200_link_rear_left", true},
  {"t200_link_vertical_right", false},
  {"t200_link_vertical_left", true},
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
  lights_{0},
  lights_trim_button_previous_{false}
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
  nh_priv_.param("effort_dead_band", effort_dead_band_, 0.005);          // Don't publish tiny thruster efforts

  // TODO simulate a rotated imu and remove this hack
  nh_priv_.param("simulation", simulation_, true);
  if (simulation_)
  {
    imu_rotation_ = tf2::Quaternion::getIdentity();
  }
  else
  {
    // TODO listen for this transform
    tf2::Quaternion imu_orientation;
    imu_orientation.setRPY(-M_PI/2, -M_PI/2, 0);
    imu_rotation_ =  imu_orientation.inverse();
  }

  // Set up all subscriptions
  baro_sub_ = nh_priv_.subscribe<orca_msgs::Barometer>("/barometer", 10, &OrcaBase::baroCallback, this);
  imu_sub_ = nh_priv_.subscribe<sensor_msgs::Imu>("/imu/data", 10, &OrcaBase::imuCallback, this);
  joy_sub_ = nh_priv_.subscribe<sensor_msgs::Joy>("/joy", 10, &OrcaBase::joyCallback, this);
  yaw_control_effort_sub_ = nh_priv_.subscribe<std_msgs::Float64>("/yaw_pid/control_effort", 10, &OrcaBase::yawControlEffortCallback, this);
  depth_control_effort_sub_ = nh_priv_.subscribe<std_msgs::Float64>("/depth_pid/control_effort", 10, &OrcaBase::depthControlEffortCallback, this);

  // Advertise all topics that we'll publish on
  control_pub_ = nh_priv_.advertise<orca_msgs::Control>("control", 1);
  marker_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray>("rviz_marker_array", 1);
  yaw_pid_enable_pub_ = nh_priv_.advertise<std_msgs::Bool>("/yaw_pid/pid_enable", 1);
  yaw_state_pub_ = nh_priv_.advertise<std_msgs::Float64>("/yaw_pid/state", 1);
  yaw_setpoint_pub_ = nh_priv_.advertise<std_msgs::Float64>("/yaw_pid/setpoint", 1);
  depth_pid_enable_pub_ = nh_priv_.advertise<std_msgs::Bool>("/depth_pid/pid_enable", 1);
  depth_state_pub_ = nh_priv_.advertise<std_msgs::Float64>("/depth_pid/state", 1);
  depth_setpoint_pub_ = nh_priv_.advertise<std_msgs::Float64>("/depth_pid/setpoint", 1);
}

// New barometer reading
void OrcaBase::baroCallback(const orca_msgs::Barometer::ConstPtr& baro_msg)
{
  depth_state_ = baro_msg->depth;
  if (!barometer_ready_)
  {
    barometer_ready_ = true;
    ROS_INFO("Barometer ready, depth %g", depth_state_);
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
    // TODO fix simulation
    imu_orientation = imu_orientation.normalize();
  }
  base_orientation_ = imu_orientation * imu_rotation_;

  double roll, pitch;
  tf2::Matrix3x3(base_orientation_).getRPY(roll, pitch, yaw_state_);

  // Compute a stability metric, used to throttle the pid controllers
  stability_ = std::min(clamp(std::cos(roll), 0.0, 1.0), clamp(std::cos(pitch), 0.0, 1.0));

  if (!imu_ready_)
  {
    imu_ready_ = true;
    ROS_INFO("IMU ready, roll %4.2f pitch %4.2f yaw %4.2f", roll, pitch, yaw_state_);
  }
}

// Result of yaw pid controller
void OrcaBase::yawControlEffortCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (mode_ == orca_msgs::Control::hold_h || mode_ == orca_msgs::Control::hold_hd)
  {
    yaw_effort_ = dead_band(msg->data * stability_, effort_dead_band_);
  }
}

// Result of depth pid controller
void OrcaBase::depthControlEffortCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (mode_ == orca_msgs::Control::hold_d || mode_ == orca_msgs::Control::hold_hd)
  {
    vertical_effort_ = dead_band(-msg->data * stability_, effort_dead_band_);
  }
}

void OrcaBase::publishYawSetpoint()
{
  if (imu_ready_)
  {
    std_msgs::Float64 setpoint;
    setpoint.data = yaw_setpoint_;
    yaw_setpoint_pub_.publish(setpoint);
  }
}

void OrcaBase::publishDepthSetpoint()
{
  if (barometer_ready_)
  {
    std_msgs::Float64 setpoint;
    setpoint.data = depth_setpoint_;
    depth_setpoint_pub_.publish(setpoint);  
  }
}

void OrcaBase::publishOdom()
{
  if (imu_ready_ && barometer_ready_)
  {
    // Publish a transform from base_link to odom with rpy (from the imu) and z (from the barometer)
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = 0;
    odom_tf.transform.translation.y = 0;
    odom_tf.transform.translation.z = -depth_state_;
    odom_tf.transform.rotation = tf2::toMsg(base_orientation_);
    tf_broadcaster_.sendTransform(odom_tf);
  }
}

void OrcaBase::publishControl()
{
  // Calc thruster efforts. Note that strafe and yaw are THRUSTER_MAX for left, THRUSTER_MIN for right.
  // Order must match the order of the <thruster> tags in the URDF.
  // 3 of the thrusters spin cw, and 3 spin ccw; see URDF for details.
  orca_msgs::Control control_msg;
  control_msg.thruster_efforts.push_back(clamp(forward_effort_ + strafe_effort_ + yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  control_msg.thruster_efforts.push_back(clamp(forward_effort_ - strafe_effort_ - yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  control_msg.thruster_efforts.push_back(clamp(forward_effort_ - strafe_effort_ + yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  control_msg.thruster_efforts.push_back(clamp(forward_effort_ + strafe_effort_ - yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  control_msg.thruster_efforts.push_back(clamp(vertical_effort_, THRUSTER_MIN, THRUSTER_MAX));
  control_msg.thruster_efforts.push_back(clamp(-vertical_effort_, THRUSTER_MIN, THRUSTER_MAX));

  // Build rviz markers
  visualization_msgs::MarkerArray markers_msg;
  for (int i = 0; i < control_msg.thruster_efforts.size(); ++i)
  {
    int32_t action = control_msg.thruster_efforts[i] == 0.0 ? action = visualization_msgs::Marker::DELETE : action = visualization_msgs::Marker::ADD;
    double scale = (THRUSTERS[i].ccw ? -control_msg.thruster_efforts[i] : control_msg.thruster_efforts[i]) / 5.0;
    double offset = scale > 0 ? -0.12 : 0.12;

    visualization_msgs::Marker marker;
    marker.header.frame_id = THRUSTERS[i].frame_id;
    marker.header.stamp = ros::Time();
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

  control_msg.mode = mode_;
  control_msg.camera_tilt = tilt_;
  control_msg.brightness = lights_;

  control_pub_.publish(control_msg);
  marker_pub_.publish(markers_msg);
}

// Change operation mode
void OrcaBase::setMode(uint8_t mode)
{
  mode_ = mode;  

  if (mode == orca_msgs::Control::hold_d || mode == orca_msgs::Control::hold_hd)
  {
    // Turn on depth pid controller
    std_msgs::Bool enable;
    enable.data = true;
    depth_pid_enable_pub_.publish(enable);
    
    // Set target depth
    depth_setpoint_ = depth_state_;
    publishDepthSetpoint();

    // Clear button state
    depth_trim_button_previous_ = false;
  }
  else
  {
    // Turn off depth pid controller
    std_msgs::Bool enable;
    enable.data = false;
    depth_pid_enable_pub_.publish(enable);
  }

  if (mode == orca_msgs::Control::hold_h || mode == orca_msgs::Control::hold_hd)
  {
    // Turn on yaw pid controller
    std_msgs::Bool enable;
    enable.data = true;
    yaw_pid_enable_pub_.publish(enable);
    
    // Set target angle
    yaw_setpoint_ = yaw_state_;
    publishYawSetpoint();

    // Clear button state
    yaw_trim_button_previous_ = false;
  }
  else
  {
    // Turn off yaw pid controller
    std_msgs::Bool enable;
    enable.data = false;
    yaw_pid_enable_pub_.publish(enable);
  }

  if (mode == orca_msgs::Control::disarmed)
  {
    forward_effort_ = yaw_effort_ = strafe_effort_ = vertical_effort_ = 0.0;
  }
}

// New input from the gamepad
void OrcaBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  constexpr double depth_hold_min = 0.05; // Hover just below the surface of the water
  constexpr double depth_hold_max = 50;   // Margin of safety

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
    if ((mode_ == orca_msgs::Control::hold_h || mode_ == orca_msgs::Control::hold_hd))
    {
      yaw_setpoint_ = joy_msg->axes[joy_axis_yaw_trim_] > 0.0 ? yaw_setpoint_ + inc_yaw_ : yaw_setpoint_ - inc_yaw_;
      publishYawSetpoint();
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
    if (mode_ == orca_msgs::Control::hold_d || mode_ == orca_msgs::Control::hold_hd)
    {
      depth_setpoint_ = clamp(joy_msg->axes[joy_axis_vertical_trim_] < 0 ? depth_setpoint_ + inc_depth_ : depth_setpoint_ - inc_depth_, 
        depth_hold_min, depth_hold_max);
      publishDepthSetpoint();
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
  if ((joy_msg->buttons[joy_button_bright_] || joy_msg->buttons[joy_button_dim_]) && !lights_trim_button_previous_)
  {
  // Rising edge
    lights_ = clamp(joy_msg->buttons[joy_button_bright_] ? lights_ + inc_lights_ : lights_ - inc_lights_, LIGHTS_MIN, LIGHTS_MAX);
    lights_trim_button_previous_ = true;
  }
  else if (!joy_msg->buttons[joy_button_bright_] && !joy_msg->buttons[joy_button_dim_] && lights_trim_button_previous_)
  {
    // Falling edge
    lights_trim_button_previous_ = false;      
  }

  // Thrusters
  forward_effort_ = dead_band(joy_msg->axes[joy_axis_forward_], input_dead_band_);
  if (mode_ == orca_msgs::Control::manual || mode_ == orca_msgs::Control::hold_d)
  {
    yaw_effort_ = dead_band(joy_msg->axes[joy_axis_yaw_], input_dead_band_);
  }
  strafe_effort_ = dead_band(joy_msg->axes[joy_axis_strafe_], input_dead_band_);
  if (mode_ == orca_msgs::Control::manual || mode_ == orca_msgs::Control::hold_h)
  {
    vertical_effort_ = dead_band(joy_msg->axes[joy_axis_vertical_], input_dead_band_);
  }
}

// Publish various messages
void OrcaBase::spinOnce()
{
  // Set target yaw
  if (mode_ == orca_msgs::Control::hold_h || mode_ == orca_msgs::Control::hold_hd)
  {
    std_msgs::Float64 yaw_state;
    yaw_state.data = yaw_state_;
    yaw_state_pub_.publish(yaw_state);
  }

  // Set target depth
  if (mode_ == orca_msgs::Control::hold_d || mode_ == orca_msgs::Control::hold_hd)
  {
    std_msgs::Float64 depth_state;
    depth_state.data = depth_state_;
    depth_state_pub_.publish(depth_state);
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
  ros::Rate r(SPIN_RATE);
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
