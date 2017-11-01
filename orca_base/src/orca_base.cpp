#include <std_msgs/Bool.h>
#include "orca_base/orca_base.h"
#include "orca_msgs/Camera.h"
#include "orca_msgs/Lights.h"
#include "orca_msgs/Thrusters.h"

// TODO move defines to yaml

// Joy message axes:
#define JOY_AXIS_YAW            0   // Left stick left/right; 1.0 is left and -1.0 is right
#define JOY_AXIS_FORWARD        1   // Left stick up/down; 1.0 is forward and -1.0 is backward
#define JOY_AXIS_STRAFE         3   // Right stick left/right; 1.0 is left and -1.0 is right
#define JOY_AXIS_VERTICAL       4   // Right stick up/down; 1.0 is ascend and -1.0 is descend
#define JOY_AXIS_YAW_TRIM       6   // Trim left/right; acts like 2 buttons; 1.0 for left and -1.0 for right
#define JOY_AXIS_VERTICAL_TRIM  7   // Trim up/down; acts like 2 buttons; 1.0 for up and -1.0 for down

// Unused axes:
// 2 Left trigger; starts from 1.0 and moves to -1.0
// 5 Right trigger; starts from 1.0 and moves to -1.0

// Joy message buttons:
#define JOY_BUTTON_DISARM       6   // View
#define JOY_BUTTON_ARM          7   // Menu
#define JOY_BUTTON_MANUAL       0   // A
#define JOY_BUTTON_STABILIZE    2   // X
#define JOY_BUTTON_DEPTH_HOLD   3   // Y
#define JOY_BUTTON_SURFACE      1   // B
#define JOY_CAMERA_TILT_DOWN    4   // Left bumper
#define JOY_CAMERA_TILT_UP      5   // Right bumper
#define JOY_LIGHTS_BRIGHT       9   // Left stick
#define JOY_LIGHTS_DIM          10  // Right stick

// Unused buttons:
// 8 Logo

// Limits
#define THRUSTER_MIN -1.0
#define THRUSTER_MAX  1.0
#define TILT_MIN     -45
#define TILT_MAX      45
#define LIGHTS_MIN    0.0
#define LIGHTS_MAX    1.0

// Trim increments
#define PI          3.14159
#define INC_YAW     PI/36
#define INC_DEPTH   0.1
#define INC_TILT    5
#define INC_LIGHTS  0.2

// Don't respond to tiny joystick movements
#define INPUT_DEAD_BAND 0.05

// Don't publish tiny thruster efforts
#define EFFORT_DEAD_BAND 0.01

// Publish messages at 100Hz
#define SPIN_RATE 100

namespace orca_base {

OrcaBase::OrcaBase(ros::NodeHandle &nh, tf::TransformListener &tf):
  nh_{nh},
  tf_{tf},
  mode_{Mode::disarmed},
  forward_effort_{0},
  yaw_effort_{0},
  strafe_effort_{0},
  vertical_effort_{0},
  tilt_{0},
  tilt_trim_button_previous_{false},
  lights_{0},
  lights_trim_button_previous_{false}
{
  // Set up all subscriptions
  baro_sub_ = nh_.subscribe<orca_msgs::Barometer>("/barometer", 10, &OrcaBase::baroCallback, this);
  imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu", 10, &OrcaBase::imuCallback, this);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &OrcaBase::joyCallback, this);
  yaw_control_effort_sub_ = nh_.subscribe<std_msgs::Float64>("/yaw_control_effort", 10, &OrcaBase::yawControlEffortCallback, this);
  depth_control_effort_sub_ = nh_.subscribe<std_msgs::Float64>("/depth_control_effort", 10, &OrcaBase::depthControlEffortCallback, this);

  // Advertise all topics that we'll publish on
  thrusters_pub_ = nh_.advertise<orca_msgs::Thrusters>("/thrusters", 1);
  camera_tilt_pub_ = nh_.advertise<orca_msgs::Camera>("/camera_tilt", 1);
  lights_pub_ = nh_.advertise<orca_msgs::Lights>("/lights", 1);
  yaw_pid_enable_pub_ = nh_.advertise<std_msgs::Bool>("/yaw_pid_enable", 1);
  yaw_state_pub_ = nh_.advertise<std_msgs::Float64>("/yaw_state", 1);
  yaw_setpoint_pub_ = nh_.advertise<std_msgs::Float64>("/yaw_setpoint", 1);
  depth_pid_enable_pub_ = nh_.advertise<std_msgs::Bool>("/depth_pid_enable", 1);
  depth_state_pub_ = nh_.advertise<std_msgs::Float64>("/depth_state", 1);
  depth_setpoint_pub_ = nh_.advertise<std_msgs::Float64>("/depth_setpoint", 1);
}

// New barometer reading
void OrcaBase::baroCallback(const orca_msgs::Barometer::ConstPtr& baro_msg)
{
  depth_state_ = baro_msg->depth;
}

// New imu reading
void OrcaBase::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  tf::Quaternion q;
  double roll, pitch, yaw;

  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  yaw_state_ = yaw;
}

// Result of yaw pid controller
void OrcaBase::yawControlEffortCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (mode_ == Mode::stabilize || mode_ == Mode::depth_hold)
  {
    yaw_effort_ = dead_band(msg->data, EFFORT_DEAD_BAND);
  }
}

// Result of depth pid controller
void OrcaBase::depthControlEffortCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (mode_ == Mode::depth_hold)
  {
    vertical_effort_ = dead_band(msg->data, EFFORT_DEAD_BAND);  
  }
}

void OrcaBase::publishYawSetpoint()
{
  std_msgs::Float64 setpoint;
  setpoint.data = yaw_setpoint_;
  yaw_setpoint_pub_.publish(setpoint);
}

void OrcaBase::publishDepthSetpoint()
{
  std_msgs::Float64 setpoint;
  setpoint.data = depth_setpoint_;
  depth_setpoint_pub_.publish(setpoint);
}

void OrcaBase::publishCameraTilt()
{
  orca_msgs::Camera msg;
  msg.tilt = tilt_;
  camera_tilt_pub_.publish(msg);
}

void OrcaBase::publishLights()
{
  orca_msgs::Lights msg;
  msg.brightness = lights_;
  lights_pub_.publish(msg);
}

// Change operation mode
void OrcaBase::setMode(Mode mode, double depth_setpoint = 0.0)
{
  // TODO mutex critical state
  mode_ = mode;  

  if (mode == Mode::depth_hold)
  {
    // Turn on depth pid controller
    std_msgs::Bool enable;
    enable.data = true;
    depth_pid_enable_pub_.publish(enable);
    
    // Set target depth
    depth_setpoint_ = depth_setpoint;
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

  if (mode == Mode::stabilize || mode == Mode::depth_hold)
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

  if (mode == Mode::disarmed)
  {
    forward_effort_ = yaw_effort_ = strafe_effort_ = vertical_effort_ = 0.0;
  }
}

// New input from the gamepad
void OrcaBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Arm/disarm
  if (joy_msg->buttons[JOY_BUTTON_DISARM])
  {
    ROS_INFO("Disarmed");
    setMode(Mode::disarmed);
  }
  else if (joy_msg->buttons[JOY_BUTTON_ARM])
  {
    ROS_INFO("Armed, manual");
    setMode(Mode::manual);
  }

  // If we're disarmed, ignore everything else
  if (mode_ == Mode::disarmed)
  {
    ROS_INFO("Disarmed, ignoring further input");    
    return;
  }

  // Mode
  if (joy_msg->buttons[JOY_BUTTON_MANUAL])
  {
    ROS_INFO("Manual");
    setMode(Mode::manual);
  }
  else if (joy_msg->buttons[JOY_BUTTON_STABILIZE])
  {
    ROS_INFO("Stabilize");
    setMode(Mode::stabilize);
  }
  else if (joy_msg->buttons[JOY_BUTTON_DEPTH_HOLD])
  {
    ROS_INFO("Depth hold");
    setMode(Mode::depth_hold, depth_state_);
  }
  else if (joy_msg->buttons[JOY_BUTTON_SURFACE])
  {
    ROS_INFO("Surface");
    setMode(Mode::depth_hold, 10.0); // TODO create notion of 'underwater' in gazebo, and set target depth to 0
  }

  // Yaw trim
  if (joy_msg->axes[JOY_AXIS_YAW_TRIM] != 0.0 && !yaw_trim_button_previous_)
  {
    // Rising edge
    if ((mode_ == Mode::stabilize || mode_ == Mode::depth_hold))
    {
      yaw_setpoint_ = joy_msg->axes[JOY_AXIS_YAW_TRIM] > 0.0 ? yaw_setpoint_ + INC_YAW : yaw_setpoint_ - INC_YAW;
      publishYawSetpoint();
    }

    yaw_trim_button_previous_ = true;
  }
  else if (joy_msg->axes[JOY_AXIS_YAW_TRIM] == 0.0 && yaw_trim_button_previous_)
  {
    // Falling edge
    yaw_trim_button_previous_ = false;
  }

  // Depth trim
  if (joy_msg->axes[JOY_AXIS_VERTICAL_TRIM] != 0.0 && !depth_trim_button_previous_)
  {
    // Rising edge
    if (mode_ == Mode::depth_hold)
    {
      // TODO clamp this to the surface
      depth_setpoint_ = joy_msg->axes[JOY_AXIS_VERTICAL_TRIM] > 0 ? depth_setpoint_ + INC_DEPTH : depth_setpoint_ - INC_DEPTH;
      publishDepthSetpoint();
    }

    depth_trim_button_previous_ = true;
  }
  else if (joy_msg->axes[JOY_AXIS_VERTICAL_TRIM] == 0.0 && depth_trim_button_previous_)
  {
    // Falling edge
    depth_trim_button_previous_ = false;
  }

  // Camera tilt
  if ((joy_msg->buttons[JOY_CAMERA_TILT_UP] || joy_msg->buttons[JOY_CAMERA_TILT_DOWN]) && !tilt_trim_button_previous_)
  {
    // Rising edge
    tilt_ = clamp(joy_msg->buttons[JOY_CAMERA_TILT_UP] ? tilt_ + INC_TILT : tilt_ - INC_TILT, TILT_MIN, TILT_MAX);
    publishCameraTilt();
    tilt_trim_button_previous_ = true;
  }
  else if (!joy_msg->buttons[JOY_CAMERA_TILT_UP] && !joy_msg->buttons[JOY_CAMERA_TILT_DOWN] && tilt_trim_button_previous_)
  {
    // Falling edge
    tilt_trim_button_previous_ = false;
  }

  // Lights
  if ((joy_msg->buttons[JOY_LIGHTS_BRIGHT] || joy_msg->buttons[JOY_LIGHTS_DIM]) && !lights_trim_button_previous_)
  {
  // Rising edge
    lights_ = clamp(joy_msg->buttons[JOY_LIGHTS_BRIGHT] ? lights_ + INC_LIGHTS : lights_ - INC_LIGHTS, LIGHTS_MIN, LIGHTS_MAX);
    publishLights();
    lights_trim_button_previous_ = true;
  }
  else if (!joy_msg->buttons[JOY_LIGHTS_BRIGHT] && !joy_msg->buttons[JOY_LIGHTS_DIM] && lights_trim_button_previous_)
  {
    // Falling edge
    lights_trim_button_previous_ = false;      
  }

  // Thrusters
  forward_effort_ = dead_band((double)joy_msg->axes[JOY_AXIS_FORWARD], INPUT_DEAD_BAND);
  if (mode_ == Mode::manual)
  {
    yaw_effort_ = dead_band((double)joy_msg->axes[JOY_AXIS_YAW], INPUT_DEAD_BAND);
  }
  strafe_effort_ = dead_band((double)joy_msg->axes[JOY_AXIS_STRAFE], INPUT_DEAD_BAND);
  if (mode_ == Mode::manual || mode_ == Mode::stabilize)
  {
    vertical_effort_ = dead_band((double)joy_msg->axes[JOY_AXIS_VERTICAL], INPUT_DEAD_BAND);
  }
}

// Called at 100Hz; publish various messages
void OrcaBase::spinOnce(const ros::TimerEvent &event)
{
  // Set target yaw
  if (mode_ == Mode::stabilize || mode_ == Mode::depth_hold)
  {
    std_msgs::Float64 yaw_state;
    yaw_state.data = yaw_state_;
    yaw_state_pub_.publish(yaw_state);
  }

  // Set target depth
  if (mode_ == Mode::depth_hold)
  {
    std_msgs::Float64 depth_state;
    depth_state.data = depth_state_;
    depth_state_pub_.publish(depth_state);
  }

  // Set thruster efforts. Note that strafe and yaw areTHRUSTER_MAX for left, THRUSTER_MIN for right.
  // Order must match the order of the <thruster> tags in the URDF.
  // 3 of the thrusters spin cw, and 3 spin ccw; see URDF for details.
  orca_msgs::Thrusters thrusters_msg;
  thrusters_msg.effort.push_back(clamp(forward_effort_ + strafe_effort_ + yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  thrusters_msg.effort.push_back(clamp(forward_effort_ - strafe_effort_ - yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  thrusters_msg.effort.push_back(clamp(forward_effort_ - strafe_effort_ + yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  thrusters_msg.effort.push_back(clamp(forward_effort_ + strafe_effort_ - yaw_effort_, THRUSTER_MIN, THRUSTER_MAX));
  thrusters_msg.effort.push_back(clamp(vertical_effort_, THRUSTER_MIN, THRUSTER_MAX));
  thrusters_msg.effort.push_back(clamp(-vertical_effort_, THRUSTER_MIN, THRUSTER_MAX));
  thrusters_pub_.publish(thrusters_msg);

  // TODO publish odometry
}

} // namespace orca_base

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orca_base");
  ros::NodeHandle nh{"~"};
  tf::TransformListener tf{nh};
  orca_base::OrcaBase orca_base{nh, tf};

  ros::Timer t = nh.createTimer(ros::Duration(1.0 / SPIN_RATE), &orca_base::OrcaBase::spinOnce, &orca_base);

  ros::spin();

  return 0;
}
