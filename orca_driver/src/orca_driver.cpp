#include "orca_driver/orca_driver.h"

// TODO move defines to yaml

#define SPIN_RATE 50

// Maestro channel assignments
#define THRUSTER_1_CHANNEL  0
#define THRUSTER_2_CHANNEL  1
#define THRUSTER_3_CHANNEL  2
// Leave a gap
#define THRUSTER_4_CHANNEL  4
#define THRUSTER_5_CHANNEL  5
#define THRUSTER_6_CHANNEL  6
// Leave a gap
#define LIGHTS_CHANNEL      8
#define TILT_CHANNEL        9
// Leave a gap
#define VOLTAGE_CHANNEL    11
#define LEAK_CHANNEL       12

namespace orca_driver {

OrcaDriver::OrcaDriver(ros::NodeHandle &nh, tf::TransformListener &tf) :
  nh_{nh},
  tf_{tf},
  thruster_channels_{THRUSTER_1_CHANNEL, THRUSTER_2_CHANNEL, THRUSTER_3_CHANNEL, THRUSTER_4_CHANNEL, THRUSTER_5_CHANNEL, THRUSTER_6_CHANNEL}
{
  // Set up subscriptions
  camera_tilt_sub_ = nh_.subscribe<orca_msgs::Camera>("/camera_tilt", 10, &OrcaDriver::cameraTiltCallback, this);
  lights_sub_ = nh_.subscribe<orca_msgs::Lights>("/lights", 10, &OrcaDriver::lightsCallback, this);
  thruster_sub_ = nh_.subscribe<orca_msgs::Thrusters>("/thrusters", 10, &OrcaDriver::thrustersCallback, this);
  
  // Advertise topics that we'll publish on
  battery_pub_ = nh_.advertise<orca_msgs::Battery>("/battery", 1);
  leak_pub_ = nh_.advertise<orca_msgs::Leak>("/leak", 1);
}

void OrcaDriver::cameraTiltCallback(const orca_msgs::Camera::ConstPtr &msg)
{
  if (maestro_.ready())
  {
    maestro_.set_pwm(TILT_CHANNEL, servo_pulse_width(-msg->tilt, -45, 45, 1100, 1900));
  }
  else
  {
    ROS_ERROR("Can't tilt camera");
  }
}

void OrcaDriver::lightsCallback(const orca_msgs::Lights::ConstPtr &msg)
{
  if (maestro_.ready())
  {
    maestro_.set_pwm(LIGHTS_CHANNEL, servo_pulse_width(-msg->brightness, 0, 100, 1100, 1900));
  }
  else
  {
    ROS_ERROR("Can't set brightness");
  }
}

void OrcaDriver::thrustersCallback(const orca_msgs::Thrusters::ConstPtr &msg)
{
  if (maestro_.ready())
  {
    for (int i = 0; i < thruster_channels_.size(); ++i)
    {
      maestro_.set_pwm(thruster_channels_[i], servo_pulse_width(msg->effort[i], -1.0, 1.0, 1100, 1900));
    }
  }
  else
  {
    ROS_ERROR("Can't operate thrusters");
  }
}

bool OrcaDriver::readBattery()
{
  float temp;
  if (maestro_.ready() && maestro_.get_analog(VOLTAGE_CHANNEL, temp))
  {
    battery_msg_.voltage = temp * 4; // Compensate for the 1/4 voltage divider
    return true;
  }
  else
  {
    ROS_ERROR("Can't read battery");
    return false;  
  }
}

bool OrcaDriver::readLeak()
{
  bool temp;
  if (maestro_.ready() && maestro_.get_digital(LEAK_CHANNEL, temp))
  {
    leak_msg_.leak_detected = temp ? 1 : 0; // TODO why isn't this a bool in the message?
    return true;
  }
  else
  {
    ROS_ERROR("Can't read leak sensor");
    return false;  
  }
}

void OrcaDriver::spinOnce()
{
  if (readBattery()) battery_pub_.publish(battery_msg_);
  if (readLeak()) leak_pub_.publish(leak_msg_);
}

// Run a bunch of pre-dive checks, return true if everything looks good
bool OrcaDriver::preDive()
{
  ROS_INFO("Running pre-dive checks...");

  if (!readBattery() || !readLeak())
  {
    maestro_.disconnect();
    return false;
  }

  ROS_INFO("Voltage %g, leak %d", battery_msg_.voltage, leak_msg_.leak_detected);

  if (leak_msg_.leak_detected)
  {
    ROS_ERROR("Leak detected");
    maestro_.disconnect();
    return false;
  }

  if (battery_msg_.voltage < 12.0) // TODO threshold move to yaml
  {
    ROS_ERROR("Battery voltage %g too low", battery_msg_.voltage);
    maestro_.disconnect();
    return false;
  }

  for (int i = 0; i < thruster_channels_.size(); ++i)
  {
    unsigned short value;
    maestro_.get_pwm(thruster_channels_[i], value);
    if (value != 1500)
    {
      ROS_ERROR("Thruster %d didn't initialize (possibly others)", i + 1);
      maestro_.disconnect();
      return false;
    }
  }

  ROS_INFO("Pre-dive checks passed");
  return true;
}

// Main entry point
bool OrcaDriver::run()
{
  std::string port = "/dev/ttyACM0"; // TODO move to yaml
  ROS_INFO("Opening port %s...", port.c_str());
  maestro_.connect(port);
  if (!maestro_.ready())
  {
    ROS_ERROR("Can't open port %s, are you root?", port.c_str());
    return false;
  }
  ROS_INFO("Port %s open", port.c_str());

  if (!preDive())
  {
    return false;
  }

  ROS_INFO("Entering main loop");  
  ros::Rate r(SPIN_RATE);
  while (ros::ok())
  {
    // Do our work
    spinOnce();
    
    // Respond to incoming messages
    ros::spinOnce();
    
    // Wait
    r.sleep();
  }

  maestro_.disconnect();
  return true;
}

} // namespace orca_driver

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "orca_driver");
  ros::NodeHandle nh{"~"};
  tf::TransformListener tf{nh}; // TODO might not need a transform listener, depends on odom implementation

  // Run the driver; in normal use the power will be yanked and run() will never return
  orca_driver::OrcaDriver orca_driver{nh, tf};
  if (orca_driver.run())
  {
    // Normal exit (Ctrl-C) during testing, ROS has already been shut down
    return 0;
  }
  else
  {
    // Orca failure during testing, hopefully we logged some decent diagnostics
    ros::shutdown();
    return -1;
  }
}