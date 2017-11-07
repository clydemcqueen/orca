#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include "orca_msgs/Barometer.h"

namespace gazebo
{

// A very simple barometer sensor plugin for underwater robotics. Usage:
//
//    <gazebo reference="base_link">
//      <sensor name="barometer_sensor" type="altimeter">
//        <update_rate>60</update_rate>
//        <plugin name="BarometerPlugin" filename="libBarometerPlugin.so">
//          <ros_topic>/barometer</ros_topic>
//          <surface>9</surface>
//          <fluid_density>1029</fluid_density>
//        </plugin>
//      </sensor>
//    </gazebo>
//
// We publish Barometer messages on <ros_topic>.
// The surface of the water is <surface> meters above the ground plane.
// The fluid density is <fluid_density> kg/m^3. Use 997 for freshwater and 1029 for seawater.
// 
// TODO: evaluate the noise model

class BarometerPlugin : public SensorPlugin
{
private:
  // Our parent sensor is an altimeter
  sensors::AltimeterSensorPtr altimeter_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // ROS node
  std::unique_ptr<ros::NodeHandle> nh_;

  // ROS publisher
  ros::Publisher baro_pub_;

  // Surface height
  double surface_;

  // Fluid density
  double fluid_density_;
  
public:

  // Called once when the plugin is loaded.
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    // Make sure that ROS is initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("ROS isn't initialized, unable to load BarometerPlugin");
      return;
    }

    // Initialize our ROS node
    nh_.reset(new ros::NodeHandle("barometer_plugin"));

    // Look for our ROS topic
    std::string ros_topic = "/barometer";
    if (sdf->HasElement("ros_topic"))
    {
      ros_topic = sdf->GetElement("ros_topic")->Get<std::string>();
    }
    ROS_INFO("BarometerPlugin will publish on ROS topic %s", ros_topic.c_str());

    // Set up ROS publisher
    baro_pub_ = nh_->advertise<orca_msgs::Barometer>(ros_topic, 1);

    // Get surface height
    surface_ = 9;
    if (sdf->HasElement("surface"))
    {
      surface_ = sdf->GetElement("surface")->Get<double>();
    }
    ROS_INFO("Water surface is %g meters above the ground plane", surface_);

    // Get water density
    constexpr double seawater_density = 1029;
    fluid_density_ = seawater_density;
    if (sdf->HasElement("fluid_density"))
    {
      fluid_density_ = sdf->GetElement("fluid_density")->Get<double>();
    }
    ROS_INFO("Fluid density is %g kg/m^3", fluid_density_);

    // Get the parent sensor
    altimeter_ = std::dynamic_pointer_cast<sensors::AltimeterSensor>(sensor);

    // Listen to the update event
    update_connection_ = altimeter_->ConnectUpdated(std::bind(&BarometerPlugin::OnUpdate, this));

    // Activate the parent sensor
    altimeter_->SetActive(true);
  }

  // The update event is broadcast at the sensor frequency, roughly 60Hz
  void OnUpdate()
  {
    constexpr double atmospheric_pressure = 1013.25;  // mbar
    constexpr double gravity = 9.80665;               // m/s^2

    orca_msgs::Barometer baro_msg;
    double depth = surface_ - altimeter_->Altitude();

    if (depth >= 0.0)
    {
      baro_msg.depth = depth;                                                             // m
      baro_msg.pressure = fluid_density_ * gravity * depth / 100 + atmospheric_pressure;  // mbar
      baro_msg.temperature = 10;                                                          // Celsius
    }
    else
    {
      baro_msg.depth = 0;
      baro_msg.pressure = atmospheric_pressure;
      baro_msg.temperature = 20;
    }
    baro_pub_.publish(baro_msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(BarometerPlugin)

} // namespace gazebo