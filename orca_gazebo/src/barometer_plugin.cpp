#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "orca_gazebo/orca_gazebo_util.h"
#include "orca_msgs/Barometer.h"

namespace gazebo
{
// A very simple barometer sensor plugin for underwater robotics. Usage:
//
//    <gazebo reference="base_link">
//      <sensor name="barometer_sensor" type="altimeter">
//        <update_rate>60</update_rate>
//        <plugin name="OrcaBarometerPlugin" filename="libOrcaBarometerPlugin.so">
//          <baro_topic>/barometer</baro_topic>
//          <pose_topic>/depth</pose_topic>
//          <fluid_density>1029</fluid_density>
//        </plugin>
//      </sensor>
//    </gazebo>
//
// Publish orca_msgs/Barometer messages on <baro_topic>.
// Publish geometry_msgs/PoseWithCovarianceStamped messages on <pose_topic>.
//
// The fluid density is <fluid_density> kg/m^3. Use 997 for freshwater and 1029 for seawater.
// The model must be spawned at the surface.

constexpr double SEAWATER_DENSITY = 1029;
constexpr double ATMOSPHERIC_PRESSURE = 101325;   // Pascals
constexpr double GRAVITY = 9.80665;               // m/s^2
constexpr double DEPTH_STDDEV = 0.01;             // m

class OrcaBarometerPlugin : public SensorPlugin
{
private:
  // Our parent sensor is an altimeter
  sensors::AltimeterSensorPtr altimeter_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // ROS node
  std::unique_ptr<ros::NodeHandle> nh_;

  // ROS publishers
  ros::Publisher baro_pub_;
  ros::Publisher pose_pub_;

  // Fluid density
  double fluid_density_;
  
public:

  // Called once when the plugin is loaded.
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    // Make sure that ROS is initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("ROS isn't initialized, unable to load OrcaBarometerPlugin");
      return;
    }

    // Initialize our ROS node
    nh_.reset(new ros::NodeHandle("barometer_plugin"));

    // Look for our ROS topic
    std::string baro_topic = "/barometer";
    if (sdf->HasElement("baro_topic"))
    {
      baro_topic = sdf->GetElement("baro_topic")->Get<std::string>();
    }
    ROS_INFO("OrcaBarometerPlugin will publish barometer message on %s", baro_topic.c_str());
    baro_pub_ = nh_->advertise<orca_msgs::Barometer>(baro_topic, 1);

    std::string pose_topic = "/depth";
    if (sdf->HasElement("pose_topic"))
    {
      pose_topic = sdf->GetElement("pose_topic")->Get<std::string>();
    }
    ROS_INFO("OrcaBarometerPlugin will publish pose message on %s", pose_topic.c_str());
    pose_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1);

    // Get water density
    fluid_density_ = SEAWATER_DENSITY;
    if (sdf->HasElement("fluid_density"))
    {
      fluid_density_ = sdf->GetElement("fluid_density")->Get<double>();
    }
    ROS_INFO("Fluid density is %g kg/m^3", fluid_density_);

    // Get the parent sensor
    altimeter_ = std::dynamic_pointer_cast<sensors::AltimeterSensor>(sensor);

    // Listen to the update event
    update_connection_ = altimeter_->ConnectUpdated(std::bind(&OrcaBarometerPlugin::OnUpdate, this));

    // Activate the parent sensor
    altimeter_->SetActive(true);
  }

  // The update event is broadcast at the sensor frequency, roughly 60Hz
  void OnUpdate()
  {

    orca_msgs::Barometer baro_msg;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.frame_id = "odom";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.covariance[14] = DEPTH_STDDEV * DEPTH_STDDEV;

    double depth = orca_gazebo::gaussianKernel(-altimeter_->Altitude(), DEPTH_STDDEV);

    // The altimeter sensor zeros out when it starts, so we don't need to subtract the distance to the surface
    if (depth >= 0.0)
    {
      baro_msg.depth = depth;
      baro_msg.pressure = fluid_density_ * GRAVITY * -altimeter_->Altitude() + ATMOSPHERIC_PRESSURE; // Pascals
      baro_msg.temperature = 10; // Celsius

      pose_msg.pose.pose.position.z = -depth; // ENU
    }
    else
    {
      baro_msg.depth = 0;
      baro_msg.pressure = ATMOSPHERIC_PRESSURE;
      baro_msg.temperature = 20;

      pose_msg.pose.pose.position.z = 0;
    }

    pose_pub_.publish(pose_msg);
    baro_pub_.publish(baro_msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(OrcaBarometerPlugin)

} // namespace gazebo