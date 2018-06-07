#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

/* Simulate a GPS sensor attached to a mast on a submersible vehicle. Generates geometry_msgs/Vector3Stamped messages.
 * Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaGPSPlugin" filename="libOrcaGPSPlugin.so">
 *        <link name="base_link">
 *          <ros_topic>/gps</ros_topic>
 *          <mast_height>0.5</mast_height>
 *          <surface>10</surface>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <ros_topic> Topic for geometry_msgs/Vector3Stamped messages. Default is /gps.
 *    <mast_height> Height of the mast above the water. Default is 0.5.
 *    <surface> How far above z=0 the surface of the water is; used to calculate depth.
 */

namespace gazebo {

class OrcaGPSPlugin : public ModelPlugin
{
private:

  physics::LinkPtr base_link_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Timer timer_;
  ros::Publisher gps_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;  // Publish tf messages
  geometry_msgs::TransformStamped gps_tf_;

  double time_above_surface_ = 0;
  double mast_height_ = 0.50;
  double surface_ = 20;

public:

  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    // Make sure that ROS is initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("ROS isn't initialized, unable to load OrcaGPSPlugin");
      return;
    }

    // Initialize our ROS node
    nh_.reset(new ros::NodeHandle("gps_plugin"));

    std::string link_name = "base_link";
    std::string ros_topic = "/gps";

    std::cout << std::endl;
    std::cout << "ORCA GPS PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default ROS topic: " << ros_topic << std::endl;
    std::cout << "Default mast height: " << mast_height_ << std::endl;
    std::cout << "Default surface: " << surface_ << std::endl;

    if (sdf->HasElement("link"))
    {
      sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

      if (linkElem->HasAttribute("name"))
      {
        linkElem->GetAttribute("name")->Get(link_name);
        std::cout << "Link name: " << link_name << std::endl;
      }

      if (linkElem->HasElement("ros_topic")) // TODO should be child of gazebo element, not link element
      {
        ros_topic = linkElem->GetElement("ros_topic")->Get<std::string>();
        std::cout << "ROS topic: " << ros_topic << std::endl;
      }

      if (linkElem->HasElement("mast_height"))
      {
        mast_height_ = linkElem->GetElement("mast_height")->Get<double>();
        std::cout << "Mast height: " << mast_height_ << std::endl;
      }

      if (linkElem->HasElement("surface")) // TODO should be child of gazebo element, not link element
      {
        surface_ = linkElem->GetElement("surface")->Get<double>();
        std::cout << "Surface: " << surface_ << std::endl;
      }
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Set up ROS publisher
    gps_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>(ros_topic, 1);

    // Create 10Hz timer
    timer_ = nh_->createTimer(ros::Duration(0.1), &OrcaGPSPlugin::TimerCallback, this);

    // Initialize transform
    gps_tf_.header.frame_id = "map";
    gps_tf_.child_frame_id = "odom";
    gps_tf_.transform.translation.x = 0;
    gps_tf_.transform.translation.y = 0;
    gps_tf_.transform.translation.z = 0;
    gps_tf_.transform.rotation.x = 0;
    gps_tf_.transform.rotation.y = 0;
    gps_tf_.transform.rotation.z = 0;
    gps_tf_.transform.rotation.w = 1;

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the ROS timer at 10Hz.
  void TimerCallback(const ros::TimerEvent &event)
  {
    // Is the GPS sensor above the water surface?
    gazebo::math::Vector3 pos = base_link_->GetWorldPose().pos;
    if (pos.z + mast_height_ > surface_)
    {
      // Do we have a satellite fix?
      if (time_above_surface_ > 5)
      {
        // TODO add noise

        // Publish gps message
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = event.current_real;
        msg.vector.x = pos.x;
        msg.vector.y = pos.y;
        msg.vector.z = 0;
        gps_pub_.publish(msg);

        // Update transform
        gps_tf_.transform.translation.x = pos.x;
        gps_tf_.transform.translation.y = pos.y;
      }
      else
      {
        time_above_surface_ += (event.current_real - event.last_real).toSec();
      }
    }
    else
    {
      time_above_surface_ = 0;
    }

    // Publish transform
    // TODO should orca_base do this???
    gps_tf_.header.stamp = event.current_real;
    tf_broadcaster_.sendTransform(gps_tf_);
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaGPSPlugin)

} // namespace gazebo
