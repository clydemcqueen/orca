#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

/* A simple buoyancy plugin. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaBuoyancyPlugin" filename="libOrcaBuoyancyPlugin.so">
 *        <fluid_density>1029</fluid_density>
 *        <surface>10</surface>
 *        <link name="base_link">
 *          <volume>0.01</volume>
 *          <center_of_volume>0 0 0.06</center_of_volume>
 *          <height>0.254</height>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <fluid_density> Density of fluid.
 *    <surface> How far above z=0 the surface of the water is; buoyancy force stops at the surface.
 *    <center_of_volume> Buoyancy force is applied to the center of volume.
 *    <volume> Total volume in m^3.
 *    <height> Height of vehicle
 *
 * Limitations:
 *    Only 1 link is supported
 *    Volume and center of volume for the link must be provided (it's not calculated)
 *    Assume vehicle density is uniform (affects behavior near the surface)
 *    Ignoring vehicle rotation (affects behavior near the surface)
 */

namespace gazebo {

class OrcaBuoyancyPlugin : public ModelPlugin
{
private:

  event::ConnectionPtr update_connection_;                // Connection to update event
  ignition::math::Vector3d gravity_;                      // Gravity vector in world frame
  physics::LinkPtr base_link_;                            // Pointer to the base link

  double fluid_density_ {1029};                           // Fluid density of seawater
  double surface_ {20};                                   // Distance to surface
  double volume_ {0.01};                                  // base_link_ volume
  ignition::math::Vector3d center_of_volume_ {0, 0, 0};   // base_link_ center of volume
  double height_{0.254};                                  // base_link_ height

public:

  // Called once when the plugin is loaded
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    // Get gravity vector
    gravity_ = model->GetWorld()->Gravity();

    // Print defaults
    std::string link_name {"base_link"};
    std::cout << std::endl;
    std::cout << "ORCA BUOYANCY PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default fluid density: " << fluid_density_ << std::endl;
    std::cout << "Default surface: " << surface_ << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default volume: " << volume_ << std::endl;
    std::cout << "Default center of volume: " << center_of_volume_ << std::endl;
    std::cout << "Default height: " << height_ << std::endl;

    // Get overrides, and print them
    if (sdf->HasElement("fluid_density"))
    {
      fluid_density_ = sdf->GetElement("fluid_density")->Get<double>();
      std::cout << "Fluid density: " << fluid_density_ << std::endl;
    }

    if (sdf->HasElement("surface"))
    {
      surface_ = sdf->GetElement("surface")->Get<double>();
      std::cout << "Surface: " << surface_ << std::endl;
    }

    if (sdf->HasElement("link"))
    {
      sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

      if (linkElem->HasAttribute("name"))
      {
        linkElem->GetAttribute("name")->Get(link_name);
        std::cout << "Link name: " << link_name << std::endl;
      }

      if (linkElem->HasElement("volume"))
      {
        volume_ = linkElem->GetElement("volume")->Get<double>();
        std::cout << "Volume: " << volume_ << std::endl;
      }

      if (linkElem->HasElement("center_of_volume"))
      {
        center_of_volume_ = linkElem->GetElement("center_of_volume")->Get<ignition::math::Vector3d>();
        std::cout << "Center of volume: " << center_of_volume_ << std::endl;
      }

      if (linkElem->HasElement("height"))
      {
        height_ = linkElem->GetElement("height")->Get<double>();
        std::cout << "Height: " << height_ << std::endl;
      }
    }

    // Get base link
    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Listen for the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OrcaBuoyancyPlugin::OnUpdate, this, _1));

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the world update start event, up to 1kHz
  void OnUpdate(const common::UpdateInfo& /*info*/)
  {
    // Get link pose in the world frame
    ignition::math::Pose3d link_frame = base_link_->WorldPose();

    if (link_frame.Pos().Z() < surface_ + height_ / 2)
    {
      // Compute buoyancy force in the world frame
      ignition::math::Vector3d buoyancy_world_frame = -fluid_density_ * volume_ * gravity_;

      // Scale buoyancy force near the surface
      if (link_frame.Pos().Z() > surface_ - height_ / 2)
      {
        double scale = (link_frame.Pos().Z() - surface_ - height_ / 2) / -height_;
        buoyancy_world_frame = buoyancy_world_frame * scale;
      }

      // Rotate buoyancy into the link frame
      ignition::math::Vector3d buoyancy_link_frame = link_frame.Rot().Inverse().RotateVector(buoyancy_world_frame);

      // Add the buoyancy force
      base_link_->AddLinkForce(buoyancy_link_frame, center_of_volume_);
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaBuoyancyPlugin)

}
