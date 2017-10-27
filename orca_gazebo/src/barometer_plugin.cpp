#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include "orca_msgs/Barometer.h"

namespace gazebo
{

// A very simple barometer sensor plugin. Usage:
//
//    <gazebo reference="base_link">
//      <sensor name="barometer_sensor" type="altimeter">
//        <update_rate>60</update_rate>
//        <plugin name="BarometerPlugin" filename="libBarometerPlugin.so">
//          <ros_topic>/barometer</ros_topic>
//        </plugin>
//      </sensor>
//    </gazebo>
//
// We publish Barometer messages on <ros_topic>.
// 
// TODO: evaluate the noise model
// TODO: how do we support the idea of a water surface?
// TODO: also publish temperature and pressure

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
    orca_msgs::Barometer baro_msg;
    baro_msg.depth = altimeter_->Altitude();
    baro_pub_.publish(baro_msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(BarometerPlugin)

} // namespace gazebo