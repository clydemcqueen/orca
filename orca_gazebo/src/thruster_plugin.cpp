#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "orca_msgs/Thruster.h"

namespace gazebo
{

// A very simple thruster plugin. Usage:
//
//    <gazebo>
//      <plugin name="ThrusterPlugin" filename="libThrusterPlugin.so"> 
//        <ros_topic>/thruster</ros_topic>
//        <thruster>
//          <force>50</force>
//          <origin xyz="0.1 0.15 0" rpy="0 ${PI/2} ${PI*3/4}"/>
//        </thruster>
//      </plugin>
//    </gazebo>
//
// We listen for ROS Thruster messages and apply thrust forces to base_link.
//
// There can the multiple <thruster> tags; the number and order of <thruster> tags must match
// the number and order of float32s in the Thruster message. Each float indicates effort and
// ranges from -1.0 (full reverse) to 1.0 (full forward).
//
//    <ros_topic> specifics the topic for Thruster messages. Default is /thrusters.
//    <force> specifies force generated this thruster in Newtons. Default is 50N. Use negative force for clockwise spin.
//    <origin> specifies thruster pose relative to base_link. Default is 0, 0, 0, 0, 0, 0.
//
// Note: the ROS URDF to SDF translation drops all fixed joints, collapsing all links into a single link.
// There are several possible workarounds:
// 1. Copy/paste the <origin> tags from each thruster <joint> tag to the <thruster> tag. (implemented)
// 2. Use non-fixed joints with motion limits. (not implemented)
// 3. Use the <dontcollapsejoints> tag. (not implemented; appears to require SDF 2.0)
//
// TODO: support nonlinear force curves, e.g., http://docs.bluerobotics.com/thrusters/t200/

class ThrusterPlugin : public ModelPlugin
{
private:
  // Pointer to our base_link
  physics::LinkPtr base_link_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // ROS node
  std::unique_ptr<ros::NodeHandle> nh_;

  // ROS subscriber
  ros::Subscriber thruster_sub_;

  // ROS callback queue
  ros::CallbackQueue callback_queue_;

  // Thread that runs the ROS message queue
  std::thread callback_thread_;

  // Model for each thruster
  struct Thruster
  {
    // Specified in the SDF, and doesn't change:
    gazebo::math::Vector3 xyz;
    gazebo::math::Vector3 rpy;
    double force; // Newtons, use negative numbers for clockwise spin

    // From the latest ROS message:
    double effort; // Range -1.0 to 1.0
  };

  // Our array of thrusters
  std::vector<Thruster> thrusters_ = {};  

  // ROS helper function that processes messages.
  void QueueThread()
  {
    ROS_INFO("ROS queue thread running");
    
    static const double timeout = 0.01;
    while (nh_->ok())
    {
      callback_queue_.callAvailable(ros::WallDuration(timeout));
    }
  }
  
public:

  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    // Make sure that ROS is initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("ROS isn't initialized, unable to load ThrusterPlugin");
      return;
    }

    // Create a ROS thread queue
    callback_thread_ = std::thread(std::bind(&ThrusterPlugin::QueueThread, this));

    // Initialize our ROS node
    nh_.reset(new ros::NodeHandle("thruster_plugin"));

    // Look for our ROS topic
    std::string ros_topic = "/thrusters";
    if (sdf->HasElement("ros_topic"))
    {
      ros_topic = sdf->GetElement("ros_topic")->Get<std::string>();
    }
    ROS_INFO("ThrusterPlugin will listen on ROS topic %s", ros_topic.c_str());

    // Subscribe to the topic
    ros::SubscribeOptions so = ros::SubscribeOptions::create<orca_msgs::Thruster>(ros_topic, 1,
        boost::bind(&ThrusterPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &callback_queue_);
    thruster_sub_ = nh_->subscribe(so);
    
    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ThrusterPlugin::OnUpdate, this, _1));
    
    // Look for <thruster> tags
    for (sdf::ElementPtr elem = sdf->GetElement("thruster"); elem; elem = elem->GetNextElement("thruster"))
    {
      Thruster t = {};

      t.force = 50;
      if (elem->HasElement("force"))
      {
        t.force = elem->GetElement("force")->Get<double>();
      }

      if (elem->HasElement("origin"))
      {
        sdf::ElementPtr origin = elem->GetElement("origin");
        if (origin->HasAttribute("xyz"))
        {
          origin->GetAttribute("xyz")->Get(t.xyz);
        }

        if (origin->HasAttribute("rpy"))
        {
          origin->GetAttribute("rpy")->Get(t.rpy);
        }
      }

      ROS_INFO("Thruster force %g xyz {%g, %g, %g} rpy {%g, %g, %g}", t.force, t.xyz.x, t.xyz.y, t.xyz.z, t.rpy.x, t.rpy.y, t.rpy.z);
      thrusters_.push_back(t);
    }

    // Thrust forces will be applied to base_link
    base_link_ = model->GetLink("base_link");
  }

  // Handle an incoming message from ROS
  void OnRosMsg(const orca_msgs::ThrusterConstPtr &msg)
  {
    for (int i = 0; i < thrusters_.size() && i < msg->effort.size(); ++i)
    {
      thrusters_[i].effort = msg->effort[i];
    }
  }

  // Called by the world update start event, up to 1000 times per second.
  void OnUpdate(const common::UpdateInfo & /*info*/)
  {
    for (Thruster t : thrusters_)
    {
      // Default thruster force points directly up
      gazebo::math::Vector3 force = {0, 0, t.effort * t.force};

      // Rotate force into place on the frame
      gazebo::math::Quaternion q = {t.rpy};
      force = q.RotateVector(force);
      
      // Match base_link's current pose
      force = base_link_->GetWorldPose().rot.RotateVector(force);

      // Apply the force to base_link
      base_link_->AddForceAtRelativePosition(force, t.xyz);
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)

}