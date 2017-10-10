#include "orca_base/orca_driver.h"

extern "C"
{
  #include <rc_usefulincludes.h>
  #include <roboticscape.h>
}

// Publish messages at 100Hz
#define SPIN_RATE 100

// TODO: this is defined twice -- move to header
#define INPUT_DEAD_BAND     0.05    // Ignore joy values smaller than this

// BlueRobotics T200 thruster PWM settings
#define THRUSTER_DEAD_BAND  25      // Thruster dead band, +/-
#define THRUSTER_OFF        1500    // Thruster off, as well as midpoint of range
#define THRUSTER_RANGE      400     // Thruster range is OFF +/- 400, or 1100 (full reverse) to 1900 (full forward)

namespace orca_driver {

// Convert effort [-1.0, 1.0] to PWM
int ThrusterValue(float input)
{
  if (input < INPUT_DEAD_BAND && input > -INPUT_DEAD_BAND)
  {
    return THRUSTER_OFF;
  }
  else if (input >= 1.0)
  {
    return THRUSTER_OFF + THRUSTER_RANGE; // Clip high
  }
  else if (input <= -1.0)
  {
    return THRUSTER_OFF - THRUSTER_RANGE; // Clip low
  }
  else
  {
    return THRUSTER_OFF + THRUSTER_RANGE * input;
  }
}

OrcaDriver::OrcaDriver(ros::NodeHandle &nh, tf::TransformListener &tf): nh_{nh}, tf_{tf}
{
  // TODO
}

void OrcaDriver::SpinOnce(const ros::TimerEvent &event)
{
  // TODO
}

} // namespace orca_driver

int main(int argc, char **argv)
{
  // Initialize all devices
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}
	
	rc_set_state(RUNNING);
	
	// Shutdown all devices
	rc_cleanup();



  // Initialize our ROS node
  ros::init(argc, argv, "orca_driver");
  ros::NodeHandle nh{"~"};
  tf::TransformListener tf{nh};
  orca_driver::OrcaDriver orca_driver{nh, tf};

  ros::Timer t = nh.createTimer(ros::Duration(1.0 / SPIN_RATE), &orca_driver::OrcaDriver::SpinOnce, &orca_driver);

  ros::spin();

  return 0;
}