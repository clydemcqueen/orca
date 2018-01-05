#include "orca_driver/orca_driver.h"

namespace orca_driver {

// Message publish rate in Hz
constexpr int SPIN_RATE = 50;

OrcaDriver::OrcaDriver(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) :
  nh_{nh},
  nh_priv_{nh_priv}
{
  nh_priv_.param<std::string>("maestro_port", maestro_port_, "/dev/ttyACM0");
  ROS_INFO("Expecting Maestro on port %s", maestro_port_.c_str());

  nh_priv_.param("num_thrusters", num_thrusters_, 6);
  ROS_INFO("Configuring for %d thrusters:", num_thrusters_);
  for (int i = 0; i < num_thrusters_; ++i)
  {
    Thruster t;
    nh_priv_.param("thruster_" + std::to_string(i + 1) + "_channel", t.channel_, i); // No checks for channel conflicts!
    nh_priv_.param("thruster_" + std::to_string(i + 1) + "_reverse", t.reverse_, false);
    thrusters_.push_back(t);
    ROS_INFO("Thruster %d on channel %d %s", i + 1, t.channel_, t.reverse_ ? "(reversed)" : "");
  }

  nh_priv_.param("lights_channel", lights_channel_, 8);
  ROS_INFO("Lights on channel %d", lights_channel_);

  nh_priv_.param("tilt_channel", tilt_channel_, 9);
  ROS_INFO("Camera servo on channel %d", tilt_channel_);

  nh_priv_.param("voltage_channel", voltage_channel_, 11); // Must be analog input
  nh_priv_.param("voltage_multiplier", voltage_multiplier_, 4.7);
  nh_priv_.param("voltage_min", voltage_min_, 12.0);
  ROS_INFO("Voltage sensor on channel %d, multiplier is %g, minimum is %g", voltage_channel_, voltage_multiplier_, voltage_min_);

  nh_priv_.param("leak_channel", leak_channel_, 12); // Must be digital input
  ROS_INFO("Leak sensor on channel %d", leak_channel_);

  // Set up subscription
  control_sub_ = nh_priv_.subscribe<orca_msgs::Control>("/orca_base/control", 10, &OrcaDriver::controlCallback, this);
  
  // Advertise topics that we'll publish on
  battery_pub_ = nh_priv_.advertise<orca_msgs::Battery>("battery", 1);
  leak_pub_ = nh_priv_.advertise<orca_msgs::Leak>("leak", 1);
}

void OrcaDriver::controlCallback(const orca_msgs::Control::ConstPtr &msg)
{
  if (maestro_.ready())
  {
    maestro_.setPWM(static_cast<uint8_t>(tilt_channel_), msg->camera_tilt_pwm);
    maestro_.setPWM(static_cast<uint8_t>(lights_channel_), msg->brightness_pwm);

    for (int i = 0; i < thrusters_.size(); ++i)
    {
      uint16_t pwm = msg->thruster_pwm[i];

      // Compensate for ESC programming errors
      if (thrusters_[i].reverse_) pwm = static_cast<uint16_t>(3000 - pwm);

      maestro_.setPWM(static_cast<uint8_t>(thrusters_[i].channel_), pwm);
    }
  }
  else
  {
    ROS_ERROR("Maestro not ready, ignoring control message");
  }
}

bool OrcaDriver::readBattery()
{
  double temp;
  if (maestro_.ready() && maestro_.getAnalog(static_cast<uint8_t>(voltage_channel_), temp))
  {
    battery_msg_.header.stamp = ros::Time::now();
    battery_msg_.voltage = temp * voltage_multiplier_;
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
  if (maestro_.ready() && maestro_.getDigital(static_cast<uint8_t>(leak_channel_), temp))
  {
    leak_msg_.header.stamp = ros::Time::now();
    leak_msg_.leak_detected = static_cast<uint8_t>(temp);
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

  ROS_INFO("Voltage is %g, leak status is %d", battery_msg_.voltage, leak_msg_.leak_detected);

  if (leak_msg_.leak_detected)
  {
    ROS_ERROR("Leak detected");
    maestro_.disconnect();
    return false;
  }

  if (battery_msg_.voltage < voltage_min_)
  {
    ROS_ERROR("Battery voltage %g is below minimum %g", battery_msg_.voltage, voltage_min_);
    maestro_.disconnect();
    return false;
  }

  for (int i = 0; i < thrusters_.size(); ++i)
  {
    uint16_t value;
    maestro_.getPWM(static_cast<uint8_t>(thrusters_[i].channel_), value);
    ROS_INFO("Thruster %d is set at %d", i + 1, value);
    if (value != 1500)
    {
      ROS_ERROR("Thruster %d didn't initialize properly (and possibly others)", i + 1);
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
  std::string port = maestro_port_;
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
  ros::NodeHandle nh{""};
  ros::NodeHandle nh_priv{"~"};

  // Run the driver; in normal use the power will be yanked and run() will never return
  orca_driver::OrcaDriver orca_driver{nh, nh_priv};
  if (orca_driver.run())
  {
    // Normal exit (Ctrl-C) during testing, ROS has already been shut down
    return 0;
  }
  else
  {
    // Orca failure during testing, hopefully we logged some decent diagnostics
    ROS_ERROR("Orca shutting down :(");
    ros::shutdown();
    return 1;
  }
}
