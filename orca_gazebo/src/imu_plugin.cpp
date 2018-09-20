#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "orca_gazebo/orca_gazebo_util.h"
#include "orca_msgs/Barometer.h"

namespace gazebo
{
// IMU sensor plugin. Supports independent Gaussian noise for magnetometers, gyros and accelerometers.
// Usage:
//
//    <gazebo reference="base_link">
//      <gravity>true</gravity>
//      <sensor name="imu_sensor" type="imu">
//        <update_rate>125</update_rate>
//        <plugin name="OrcaIMUPlugin" filename="libOrcaIMUPlugin.so" />
//      </sensor>
//    </gazebo>
//
// Publishes sensor_msgs::Imu messages on /imu/data.

// TODO publish sensor_msgs::MagneticField messages on /imu/mag
// TODO remove orientation from sensor_msgs::Imu messages, and publish on /imu/data_raw instead
// TODO allow use of rotated links like imu_link

// https://github.com/ros-drivers/phidgets_drivers/blob/kinetic/phidgets_imu/src/imu_ros_i.cpp
constexpr double G = 9.80665;
constexpr double ACCEL_STDDEV = 300.0 * 1e-6 * G;
constexpr double GYRO_STDDEV = 0.02 * (M_PI / 180.0);
constexpr double MAG_STDDEV = 0.095 * (M_PI / 180.0);
constexpr double ORIENTATION_STDDEV = MAG_STDDEV; // Temporary hack

class OrcaIMUPlugin : public SensorPlugin
{
private:
  // Our parent sensor is an imu sensor
  sensors::ImuSensorPtr sensor_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // ROS node
  std::unique_ptr<ros::NodeHandle> nh_;

  // ROS publisher
  ros::Publisher imu_pub_;

  // ROS message
  sensor_msgs::Imu imu_msg_;

public:

  // Called once when the plugin is loaded.
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    // Make sure that ROS is initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("ROS isn't initialized, unable to load OrcaIMUPlugin");
      return;
    }

    // Initialize our ROS node
    nh_.reset(new ros::NodeHandle("imu_plugin"));

    // Prepare to publish
    imu_pub_ = nh_->advertise<sensor_msgs::Imu>("/imu/data", 1);
    imu_msg_.header.frame_id = "base_link";
    imu_msg_.linear_acceleration_covariance[0] = ACCEL_STDDEV * ACCEL_STDDEV;
    imu_msg_.linear_acceleration_covariance[4] = ACCEL_STDDEV * ACCEL_STDDEV;
    imu_msg_.linear_acceleration_covariance[8] = ACCEL_STDDEV * ACCEL_STDDEV;
    imu_msg_.angular_velocity_covariance[0] = GYRO_STDDEV * GYRO_STDDEV;
    imu_msg_.angular_velocity_covariance[4] = GYRO_STDDEV * GYRO_STDDEV;
    imu_msg_.angular_velocity_covariance[8] = GYRO_STDDEV * GYRO_STDDEV;
    imu_msg_.orientation_covariance[0] = ORIENTATION_STDDEV * ORIENTATION_STDDEV;
    imu_msg_.orientation_covariance[4] = ORIENTATION_STDDEV * ORIENTATION_STDDEV;
    imu_msg_.orientation_covariance[8] = ORIENTATION_STDDEV * ORIENTATION_STDDEV;

    // Get the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(sensor);

    // Listen to the update event
    update_connection_ = sensor_->ConnectUpdated(std::bind(&OrcaIMUPlugin::OnUpdate, this));

    // Activate the parent sensor
    sensor_->SetActive(true);
  }

  // The update event is broadcast at the sensor frequency
  void OnUpdate()
  {
    // Only publish if there are subscribers
    if(imu_pub_.getNumSubscribers() > 0)
    {
      imu_msg_.header.stamp = ros::Time::now();

      // Get accel and gyro readings in the sensor frame (base_link)
      ignition::math::Vector3d linear_acceleration = sensor_->LinearAcceleration(true); // True: don't add noise
      ignition::math::Vector3d angular_velocity = sensor_->AngularVelocity(true);       // True: don't add noise

      // Get orientation in the reference frame -- set on boot to the world frame (odom)
      ignition::math::Quaterniond orientation = sensor_->Orientation();

      // TODO turn orientation into a magnetometer reading -- tricky

      // Add noise
      orca_gazebo::addNoise(ACCEL_STDDEV, linear_acceleration);
      orca_gazebo::addNoise(GYRO_STDDEV, angular_velocity);
      orca_gazebo::addNoise(ORIENTATION_STDDEV, orientation);

      // Copy to message
      orca_gazebo::ignition2msg(linear_acceleration, imu_msg_.linear_acceleration);
      orca_gazebo::ignition2msg(angular_velocity, imu_msg_.angular_velocity);
      orca_gazebo::ignition2msg(orientation, imu_msg_.orientation);

      imu_pub_.publish(imu_msg_);
    }
  }
};

GZ_REGISTER_SENSOR_PLUGIN(OrcaIMUPlugin)

} // namespace gazebo