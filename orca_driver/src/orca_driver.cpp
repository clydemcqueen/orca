#include "orca_driver/orca_util.h"
#include "orca_driver/orca_driver.h"
#include "orca_msgs/Barometer.h"
#include "orca_msgs/Voltage.h"

#define SPIN_RATE   50
#define BLINK_RATE  4

#define INPUT_DEAD_BAND 0.05

// BlueRobotics T200 thruster PWM settings
#define THRUSTER_DEAD_BAND  25      // Thruster dead band, +/-
#define THRUSTER_OFF        1500    // Thruster off, as well as midpoint of range
#define THRUSTER_RANGE      400     // Thruster range is OFF +/- 400, or 1100 (full reverse) to 1900 (full forward)

// PWM channel assignments
#define PWM_THRUSTER1     1
#define PWM_THRUSTER2     2
#define PWM_THRUSTER3     3
#define PWM_THRUSTER4     4
#define PWM_THRUSTER5     5
#define PWM_THRUSTER6     6
#define PWM_LIGHTS        7
#define PWM_CAMERA_TILT   8

// TODO resolve I2C contention problem
#undef ENABLE_IMU

namespace orca_driver {

// The IMU requires a 'void (*)()' callback, so create an object wrapper
OrcaDriver* g_driver = nullptr;
void imu_callback()
{
  g_driver->publishIMU();
}

// Convert effort [-1.0, 1.0] to PWM
int thrusterValue(float input)
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
  
  // Set up all subscriptions
  camera_tilt_sub_ = nh_.subscribe<orca_msgs::Camera>("/camera_tilt", 10, &OrcaDriver::cameraTiltCallback, this);

  // Advertise all topics that we'll publish on
  barometer_pub_ = nh_.advertise<orca_msgs::Barometer>("/internal_barometer", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 1);
  voltage_pub_ = nh_.advertise<orca_msgs::Voltage>("/voltage", 1);
}

void OrcaDriver::cameraTiltCallback(const orca_msgs::Camera::ConstPtr &msg)
{
  // Input is -1.0 (down 45deg) to 1.0 (up 45deg); output is 1100us to 1900us
  // TODO rename thrusterValue, move to util
  rc_send_servo_pulse_us(PWM_CAMERA_TILT, thrusterValue(-msg->tilt));
}

// Blink an LED to indicate that we're alive
void OrcaDriver::heartbeat()
{
    loop_counter_++;
    if (loop_counter_ >= SPIN_RATE / BLINK_RATE / 2)
    {
      loop_counter_ = 0;
      led_on_ = !led_on_;
      rc_set_led(GREEN, led_on_ ? ON : OFF);
    }
}

void OrcaDriver::publishBarometer()
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

void OrcaDriver::publishIMU()
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

void OrcaDriver::publishVoltage()
{
  orca_msgs::Voltage msg;
  msg.voltage = rc_dc_jack_voltage();
  voltage_pub_.publish(msg);
}

void OrcaDriver::spinOnce()
{
  heartbeat();
  publishBarometer();
  // We're using the DMP (digital motion processor), so PublishIMU is called via interrupts
  publishVoltage();
}

int OrcaDriver::run()
{
  // Initialize the hardware
  if (rc_initialize())
  {
    ROS_ERROR("rc_initialize failed, are you root?");
    return -1;
  }

  rc_set_led(RED, ON);
  
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
  rc_enable_servo_power_rail(); // Be sure to clip the power wire on all ESCs!
  rc_send_servo_pulse_us(PWM_CAMERA_TILT, 1300);
  rc_usleep(100000L);
  rc_send_servo_pulse_us(PWM_CAMERA_TILT, 1700);
  rc_usleep(100000L);
  rc_send_servo_pulse_us(PWM_CAMERA_TILT, 1500);
  rc_usleep(100000L);
  
  // Initialize the ESCs
  // TODO
  rc_send_servo_pulse_us(PWM_THRUSTER1, 1500);
  rc_usleep(1000000L);
  
  ROS_INFO("Hardware initialized");
  rc_set_led(RED, OFF);
  
#ifdef ENABLE_IMU
  // Set the IMU callback; calls will start immediately
  g_driver = this;
  rc_set_imu_interrupt_func(&imu_callback);
#endif
  
  // Run our loop
  ros::Rate r(SPIN_RATE);
  while (rc_get_state() != EXITING)
  {
    // Do our work
    spinOnce();
    
    // Respond to incoming messages
    ros::spinOnce();
    
    // Wait
    r.sleep();
  }

  ROS_INFO("Somebody hit Ctrl-C; cleaning up");

  // Turn off ESCs
  // TODO
  
  // Turn off servos
  rc_disable_servo_power_rail();
  
  // Turn off leak detector
  // TODO
  
  // Turn off external barometer
  // TODO
  
#ifdef ENABLE_IMU
  // Turn off IMU
  rc_stop_imu_interrupt_func();
  g_driver = nullptr;
  rc_power_off_imu();
#endif

  // Turn off internal barometer
  rc_power_off_barometer();
  
  // Turn off the hardware 
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

  int rc = orca_driver.run();
  
  // Shutdown ROS
  ros::shutdown();
  
  return rc;
}
