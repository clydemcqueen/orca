#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "orca_gazebo/orca_gazebo_util.h"

/* Publish a ground truth pose for a link.
 * Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaGroundTruthPlugin" filename="libOrcaGroundTruthPlugin.so">
 *        <link name="base_link">
 *          <pose_topic>/ground_truth</pose_topic>
 *          <surface>10</surface>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <pose_topic> Topic for geometry_msgs/PoseStamped messages. Default is /ground_truth.
 *    <surface> How far above z=0 the surface of the water is; used to calculate depth. Default is 20.
 */

namespace gazebo {

class OrcaGroundTruthPlugin : public ModelPlugin
{
private:

  physics::LinkPtr base_link_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Timer timer_;
  ros::Publisher ground_truth_pub_;

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
      ROS_FATAL_STREAM("ROS isn't initialized, unable to load OrcaGroundTruthPlugin");
      return;
    }

    // Initialize our ROS node
    nh_.reset(new ros::NodeHandle("ground_truth_plugin"));

    std::string link_name = "base_link";
    std::string pose_topic = "/ground_truth";

    std::cout << std::endl;
    std::cout << "ORCA GROUND TRUTH PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default pose topic: " << pose_topic << std::endl;
    std::cout << "Default surface: " << surface_ << std::endl;

    if (sdf->HasElement("link"))
    {
      sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

      if (linkElem->HasAttribute("name"))
      {
        linkElem->GetAttribute("name")->Get(link_name);
        std::cout << "Link name: " << link_name << std::endl;
      }
    }

    if (sdf->HasElement("pose_topic"))
    {
      pose_topic = sdf->GetElement("pose_topic")->Get<std::string>();
      std::cout << "Pose topic: " << pose_topic << std::endl;
    }

    if (sdf->HasElement("surface"))
    {
      surface_ = sdf->GetElement("surface")->Get<double>();
      std::cout << "Surface: " << surface_ << std::endl;
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Set up ROS publisher
    ground_truth_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(pose_topic, 1);

    // Create timer
    timer_ = nh_->createTimer(ros::Duration(0.1), &OrcaGroundTruthPlugin::TimerCallback, this); // TODO spin rate should be a parameter

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the ROS timer.
  void TimerCallback(const ros::TimerEvent &event)
  {
    gazebo::math::Pose pose = base_link_->GetWorldPose();

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = event.current_real;
    msg.pose.position.x = pose.pos.x;
    msg.pose.position.y = pose.pos.y;
    msg.pose.position.z = pose.pos.z - surface_;
    msg.pose.orientation.x = pose.rot.x;
    msg.pose.orientation.y = pose.rot.y;
    msg.pose.orientation.z = pose.rot.z;
    msg.pose.orientation.w = pose.rot.w;

    ground_truth_pub_.publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaGroundTruthPlugin)

} // namespace gazebo
