#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "orca_gazebo/orca_gazebo_util.h"

/* Simulate a GPS sensor attached to a mast on a submersible vehicle. Generates geometry_msgs/Vector3Stamped messages.
 * Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaGPSPlugin" filename="libOrcaGPSPlugin.so">
 *        <link name="base_link">
 *          <pose_topic>/gps</pose_topic>
 *          <mast_height>0.5</mast_height>
 *          <surface>10</surface>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <pose_topic> Topic for geometry_msgs/PoseWithCovarianceStamped messages. Default is /gps.
 *    <mast_height> Height of the mast above the water. Default is 0.5.
 *    <surface> How far above z=0 the surface of the water is; used to calculate depth.
 */

namespace gazebo {

// https://www.gps.gov/systems/gps/performance/accuracy/
constexpr double GPS_STDDEV = 1.891 / 2;

class OrcaGPSPlugin : public ModelPlugin
{
private:

  physics::LinkPtr base_link_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Timer timer_;
  ros::Publisher gps_pub_;

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
    std::string pose_topic = "/gps";

    std::cout << std::endl;
    std::cout << "ORCA GPS PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default pose topic: " << pose_topic << std::endl;
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

      if (linkElem->HasElement("pose_topic")) // TODO should be child of gazebo element, not link element
      {
        pose_topic = linkElem->GetElement("pose_topic")->Get<std::string>();
        std::cout << "Pose topic: " << pose_topic << std::endl;
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
    gps_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1);

    // Create timer
    timer_ = nh_->createTimer(ros::Duration(1.0), &OrcaGPSPlugin::TimerCallback, this);

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the ROS timer.
  void TimerCallback(const ros::TimerEvent &event)
  {
    // Is the GPS sensor above the water surface?
    ignition::math::Vector3d pos = base_link_->WorldPose().Pos();
    if (pos.Z() + mast_height_ > surface_)
    {
      // Do we have a satellite fix? Assume it takes 5 seconds
      if (time_above_surface_ > 5)
      {
        // Publish pose
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = event.current_real;
        msg.pose.pose.position.x = orca_gazebo::gaussianKernel(pos.X(), GPS_STDDEV);
        msg.pose.pose.position.y = orca_gazebo::gaussianKernel(pos.Y(), GPS_STDDEV);
        msg.pose.pose.position.z = orca_gazebo::gaussianKernel(pos.Z() - surface_, GPS_STDDEV);
        msg.pose.covariance[0] = GPS_STDDEV * GPS_STDDEV;
        msg.pose.covariance[7] = GPS_STDDEV * GPS_STDDEV;
        gps_pub_.publish(msg);
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
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaGPSPlugin)

} // namespace gazebo
