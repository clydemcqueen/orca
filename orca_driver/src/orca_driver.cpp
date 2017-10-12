#include "orca_driver/orca_driver.h"
#include "orca_msgs/Barometer.h"
#include "orca_msgs/Voltage.h"

#define SPIN_RATE   100   // Publish messages at 100Hz
#define BLINK_RATE  4     // Blink an LED while the loop is running

#define INPUT_DEAD_BAND 0.05

// BlueRobotics T200 thruster PWM settings
#define THRUSTER_DEAD_BAND  25      // Thruster dead band, +/-
#define THRUSTER_OFF        1500    // Thruster off, as well as midpoint of range
#define THRUSTER_RANGE      400     // Thruster range is OFF +/- 400, or 1100 (full reverse) to 1900 (full forward)

namespace orca_driver {

// The IMU requires a 'void (*)()' callback, so create an object wrapper
OrcaDriver* g_driver = nullptr;
void imu_callback()
{
  g_driver->PublishIMU();
}

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

OrcaDriver::OrcaDriver(ros::NodeHandle &nh, tf::TransformListener &tf) :
  nh_{nh},
  tf_{tf},
  loop_counter_{0},
  led_on_{false}
{
  imu_msg_.header.frame_id = "base_link";
  // TODO imu_msg_.orientation_covariance
  // TODO img_msg_.angular_velocity_covariance
  // TODO imu_msg_.linear_acceleration_covariance
  
  barometer_pub_ = nh_.advertise<orca_msgs::Barometer>("/internal_barometer", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 1);
  voltage_pub_ = nh_.advertise<orca_msgs::Voltage>("/voltage", 1);
}

// Blink an LED to indicate that we're alive
void OrcaDriver::Heartbeat()
{
    loop_counter_++;
    if (loop_counter_ > SPIN_RATE / BLINK_RATE / 2)
    {
      loop_counter_ = 0;
      led_on_ = !led_on_;
      rc_set_led(GREEN, led_on_ ? ON : OFF);
    }
}

void OrcaDriver::PublishBarometer()
{
  // Read the sensor via I2C
  if(rc_read_barometer() < 0)
  {
    ROS_WARN("Can't read barometer");
    return;
  }

  orca_msgs::Barometer msg;
  msg.temperature = rc_bmp_get_temperature(); // Celsius
  msg.pressure = rc_bmp_get_pressure_pa() / 100000.0; // Bar
  barometer_pub_.publish(msg);
}

void OrcaDriver::PublishIMU()
{
  // We're called when the data is ready and sitting in imu_buffer_
  imu_msg_.header.stamp = ros::Time::now();
  imu_msg_.orientation.w = imu_buffer_.dmp_quat[QUAT_W];
  imu_msg_.orientation.x = imu_buffer_.dmp_quat[QUAT_X];
  imu_msg_.orientation.y = imu_buffer_.dmp_quat[QUAT_Y];
  imu_msg_.orientation.z = imu_buffer_.dmp_quat[QUAT_Z];
  imu_msg_.angular_velocity.x = imu_buffer_.gyro[0];
  imu_msg_.angular_velocity.y = imu_buffer_.gyro[1];
  imu_msg_.angular_velocity.z = imu_buffer_.gyro[2];
  imu_msg_.linear_acceleration.x = imu_buffer_.accel[0];
  imu_msg_.linear_acceleration.y = imu_buffer_.accel[1];
  imu_msg_.linear_acceleration.z = imu_buffer_.accel[2];
  imu_pub_.publish(imu_msg_);
}

void OrcaDriver::PublishVoltage()
{
  orca_msgs::Voltage msg;
  msg.voltage = rc_dc_jack_voltage();
  voltage_pub_.publish(msg);
}

void OrcaDriver::SpinOnce(const ros::TimerEvent &event)
{
  if (rc_get_state() == EXITING)
  {
    ROS_INFO("Cleaning up");

    // Shutdown ROS
    ros::shutdown();
  }
  else
  {
    Heartbeat();
    PublishBarometer();
    // We're using the DMP (digital motion processor), so PublishIMU is called via interrupts
    PublishVoltage();
  }
}

int OrcaDriver::Run()
{
  // Initialize hardware
  if (rc_initialize())
  {
    ROS_ERROR("rc_initialize failed, are you root?");
    return -1;
  }
  
  // Initialize the internal barometer
  if(rc_initialize_barometer(BMP_OVERSAMPLE_4, BMP_FILTER_16) < 0)
  {
    ROS_ERROR("rc_initialize_barometer failed");
    return -1;
  }

#ifdef ENABLE_IMU
  // Initialize the IMU
  rc_imu_config_t conf = rc_default_imu_config();
  conf.enable_magnetometer = 1; // Turn on the compass
  conf.dmp_sample_rate = SPIN_RATE; // Sample at our spin rate
  if (rc_initialize_imu_dmp(&imu_buffer_, conf))
  {
    ROS_ERROR("rc_initialize_imu_dmp failed");
    return -1;
  }
#endif
  
  // Initialize the external barometer
  // TODO
  
  // Initialize the leak detector
  // TODO
  
  // Initialize the servos
  // TODO
  
  // Initialize the ESCs
  // TODO

  ROS_INFO("Hardware initialized");
  
#ifdef ENABLE_IMU
  // Set the IMU callback; calls will start immediately
  g_driver = this;
  rc_set_imu_interrupt_func(&imu_callback);
#endif
  
  // Run our combined ROS and robotics cape loop
  ros::Timer t = nh_.createTimer(ros::Duration(1.0 / SPIN_RATE), &OrcaDriver::SpinOnce, this);
  ros::spin();
  
  // Restore hardware to a clean state
#ifdef ENABLE_IMU
  rc_stop_imu_interrupt_func();
  g_driver = nullptr;
  rc_power_off_imu();
#endif
  rc_power_off_barometer();
  rc_cleanup();

  return 0;
}

} // namespace orca_driver

int main(int argc, char **argv)
{
  // Initialize ROS _before_ firing up the robotics cape
  ros::init(argc, argv, "orca_driver");
  ros::NodeHandle nh{"~"};
  tf::TransformListener tf{nh};
  orca_driver::OrcaDriver orca_driver{nh, tf};
  ROS_INFO("ROS initialized");

  return orca_driver.Run();
}