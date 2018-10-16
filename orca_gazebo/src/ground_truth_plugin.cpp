#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "orca_gazebo/orca_gazebo_util.h"

/* Publish a ground truth pose for a link.
 * Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaGroundTruthPlugin" filename="libOrcaGroundTruthPlugin.so">
 *        <link name="base_link">
 *          <odom_topic>/ground_truth</odom_topic>
 *          <surface>10</surface>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <odom_topic> Topic for nav_msgs/Odometry messages. Default is /ground_truth.
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
    std::string odom_topic = "/ground_truth";

    std::cout << std::endl;
    std::cout << "ORCA GROUND TRUTH PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default odom topic: " << odom_topic << std::endl;
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

    if (sdf->HasElement("odom_topic"))
    {
      odom_topic = sdf->GetElement("odom_topic")->Get<std::string>();
      std::cout << "Pose topic: " << odom_topic << std::endl;
    }

    if (sdf->HasElement("surface"))
    {
      surface_ = sdf->GetElement("surface")->Get<double>();
      std::cout << "Surface: " << surface_ << std::endl;
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Set up ROS publisher
    ground_truth_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic, 1);

    // Create timer
    timer_ = nh_->createTimer(ros::Duration(0.1), &OrcaGroundTruthPlugin::TimerCallback, this); // TODO spin rate should be a parameter

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the ROS timer.
  void TimerCallback(const ros::TimerEvent &event)
  {
    // Pose in world frame
    ignition::math::Pose3d pose = base_link_->WorldPose();

    // Linear velo in world frame
    ignition::math::Vector3d linear_vel = base_link_->WorldLinearVel();

    // TODO get angular velo in odom frame

    // TODO set covar (very small, and fixed)

    nav_msgs::Odometry msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = event.current_real;
    msg.child_frame_id = "base_link"; // TODO child frame id should be a parameter
    msg.pose.pose.position.x = pose.Pos().X();
    msg.pose.pose.position.y = pose.Pos().Y();;
    msg.pose.pose.position.z = pose.Pos().Z() - surface_;
    msg.pose.pose.orientation.x = pose.Rot().X();
    msg.pose.pose.orientation.y = pose.Rot().Y();
    msg.pose.pose.orientation.z = pose.Rot().Z();
    msg.pose.pose.orientation.w = pose.Rot().W();
    msg.twist.twist.linear.x = linear_vel.X();
    msg.twist.twist.linear.y = linear_vel.Y();
    msg.twist.twist.linear.z = linear_vel.Z();

    ground_truth_pub_.publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaGroundTruthPlugin)

} // namespace gazebo
