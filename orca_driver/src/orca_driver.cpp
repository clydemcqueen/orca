#include "orca_driver/orca_driver.h"

template<class T>
constexpr const T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}
  
constexpr const uint16_t servo_pulse_width(const double v, const double v_min, const double v_max, const uint16_t pwm_min, const uint16_t pwm_max)
{
  return clamp(static_cast<uint16_t>(pwm_min + static_cast<double>(pwm_max - pwm_min) / (v_max - v_min) * (v - v_min)), pwm_min, pwm_max);
}

namespace orca_driver {

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

  nh_priv_.param("thruster_limit", thruster_limit_, 1.0);
  ROS_INFO("Thruster effort limited to %g", thruster_limit_);

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

  nh_priv_.param("spin_rate", spin_rate_, 50);
  ROS_INFO("Publishing messages at %d Hz", spin_rate_);

  // Set up subscriptions
  camera_tilt_sub_ = nh_priv_.subscribe<orca_msgs::Camera>("/orca_base/camera_tilt", 10, &OrcaDriver::cameraTiltCallback, this);
  lights_sub_ = nh_priv_.subscribe<orca_msgs::Lights>("/orca_base/lights", 10, &OrcaDriver::lightsCallback, this);
  thrusters_sub_ = nh_priv_.subscribe<orca_msgs::Thrusters>("/orca_base/thrusters", 10, &OrcaDriver::thrustersCallback, this);
  
  // Advertise topics that we'll publish on
  battery_pub_ = nh_priv_.advertise<orca_msgs::Battery>("battery", 1);
  leak_pub_ = nh_priv_.advertise<orca_msgs::Leak>("leak", 1);
}

void OrcaDriver::cameraTiltCallback(const orca_msgs::Camera::ConstPtr &msg)
{
  if (maestro_.ready())
  {
    uint16_t pwm = servo_pulse_width(-msg->tilt, -45, 45, 1100, 1900);
    ROS_DEBUG("Set tilt pulse width to %d", pwm);
    maestro_.setPWM(static_cast<uint8_t>(tilt_channel_), pwm);
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
    uint16_t pwm = servo_pulse_width(msg->brightness, 0, 100, 1100, 1900);
    ROS_DEBUG("Set lights pulse width to %d", pwm);
    maestro_.setPWM(static_cast<uint8_t>(lights_channel_), pwm);
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
    for (int i = 0; i < thrusters_.size(); ++i)
    {
      double effort = msg->effort[i];
      effort = clamp(effort, -thruster_limit_, thruster_limit_);

      // Compensate for ESC programming errors
      if (thrusters_[i].reverse_ )
      {
        effort = -effort;
      }

      maestro_.setPWM(static_cast<uint8_t>(thrusters_[i].channel_), servo_pulse_width(effort, -1.0, 1.0, 1100, 1900));
    }
  }
  else
  {
    ROS_ERROR("Can't operate thrusters");
  }
}

bool OrcaDriver::readBattery()
{
  double temp;
  if (maestro_.ready() && maestro_.getAnalog(static_cast<uint8_t>(voltage_channel_), temp))
  {
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
    leak_msg_.leak_detected = temp;
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
  ros::Rate r(spin_rate_);
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
