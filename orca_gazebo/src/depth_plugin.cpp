#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include "orca_base/Depth.h"

namespace gazebo
{

// A very simple depth sensor plugin. Usage:
//
//    <gazebo reference="base_link">
//      <sensor name="depth_sensor" type="altimeter">
//        <update_rate>60</update_rate>
//        <plugin name="DepthPlugin" filename="libDepthPlugin.so">
//          <ros_topic>/depth</ros_topic>
//        </plugin>
//      </sensor>
//    </gazebo>
//
// We publish Depth messages on <ros_topic>.
// 
// TODO: evaluate the noise model
// TODO: how do we support the idea of a water surface?

class DepthPlugin : public SensorPlugin
{
private:
  // Our parent sensor is an altimeter
  sensors::AltimeterSensorPtr altimeter_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // ROS node
  std::unique_ptr<ros::NodeHandle> nh_;

  // ROS publisher
  ros::Publisher depth_pub_;
  
public:

  // Called once when the plugin is loaded.
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    // Make sure that ROS is initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("ROS isn't initialized, unable to load DepthPlugin");
      return;
    }

    // Initialize our ROS node
    nh_.reset(new ros::NodeHandle("depth_plugin"));

    // Look for our ROS topic
    std::string ros_topic = "/depth";
    if (sdf->HasElement("ros_topic"))
    {
      ros_topic = sdf->GetElement("ros_topic")->Get<std::string>();
    }
    ROS_INFO("DepthPlugin will publish on ROS topic %s", ros_topic.c_str());

    // Set up ROS publisher
    depth_pub_ = nh_->advertise<orca_base::Depth>(ros_topic, 1);

    // Get the parent sensor
    altimeter_ = std::dynamic_pointer_cast<sensors::AltimeterSensor>(sensor);

    // Listen to the update event
    update_connection_ = altimeter_->ConnectUpdated(std::bind(&DepthPlugin::OnUpdate, this));

    // Activate the parent sensor
    altimeter_->SetActive(true);
  }

  // The update event is broadcast at the sensor frequency, roughly 60Hz
  void OnUpdate()
  {
    orca_base::Depth depth_msg;
    depth_msg.depth = altimeter_->Altitude();
    depth_pub_.publish(depth_msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(DepthPlugin)

} // namespace gazebo