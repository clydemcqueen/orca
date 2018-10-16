#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

/* A simple drag plugin. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaDragPlugin" filename="libOrcaDragPlugin.so">
 *        <link name="base_link">
 *          <center_of_mass>0 0 -0.2</center_of_mass>
 *          <tether_attach>-0.5, -0.4, 0</tether_attach>
 *          <linear_drag>10 20 30</linear_drag>
 *          <angular_drag>5 10 15</angular_drag>
 *          <tether_drag>4</tether_drag>
 *          <surface>10</surface>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <center_of_mass> Drag force is applied to the center of mass.
 *    <tether_attach> Relative position of tether attachment.
 *    <linear_drag> Linear drag constants. See default calculation.
 *    <angular_drag> Angular drag constants. See defaut calculation.
 *    <tether_drag> Tether drag constant. See default calculation.
 *    <surface> How far above z=0 the surface of the water is; used to calculate depth.
 *
 * Limitations:
 *    Tether drag is modeled only in x
 */

namespace gazebo {

/* drag = 0.5 * density * area * velocity^2 * coefficient
 *
 * The drag coefficient for a box is 1.0, so we'll use 0.9 for the ROV.
 * The drag coefficient for an unfaired tether is 1.2.
 *
 * The ROV constants below capture all but velocity:
 * constant = 0.5 * density * area * coefficient
 *
 * The tether constant below captures all but depth and velocity:
 * constant = 0.5 * density * width * coefficient
 */

constexpr double FLUID_DENSITY = 1029;    // Fluid density of seawater
constexpr double ROV_DIM_X = 0.457;       // Length
constexpr double ROV_DIM_Y = 0.338;       // Width
constexpr double ROV_DIM_Z = 0.254;       // Height
constexpr double ROV_AREA_X = ROV_DIM_Y * ROV_DIM_Z;  // Fore, aft area
constexpr double ROV_AREA_Y = ROV_DIM_X * ROV_DIM_Z;  // Top, bottom area
constexpr double ROV_AREA_Z = ROV_DIM_X * ROV_DIM_Y;  // Port, starboard area
constexpr double ROV_DRAG_COEFFICIENT_X = 0.8;
constexpr double ROV_DRAG_COEFFICIENT_Y = 0.95;
constexpr double ROV_DRAG_COEFFICIENT_Z = 0.95;
constexpr double ROV_LINEAR_DRAG_X = 0.5 * FLUID_DENSITY * ROV_AREA_X * ROV_DRAG_COEFFICIENT_X;
constexpr double ROV_LINEAR_DRAG_Y = 0.5 * FLUID_DENSITY * ROV_AREA_Y * ROV_DRAG_COEFFICIENT_Y;
constexpr double ROV_LINEAR_DRAG_Z = 0.5 * FLUID_DENSITY * ROV_AREA_Z * ROV_DRAG_COEFFICIENT_Z;
constexpr double ANGULAR_DRAG_X = ROV_LINEAR_DRAG_X / 2; // A hack
constexpr double ANGULAR_DRAG_Y = ROV_LINEAR_DRAG_Y / 2;
constexpr double ANGULAR_DRAG_Z = ROV_LINEAR_DRAG_Z / 2; // TODO these are wrong, see revised cals in orca_mission.cpp
constexpr double TETHER_DIAM = 0.008;
constexpr double TETHER_DRAG_COEFFICIENT = 1.1;
constexpr double TETHER_DRAG = 0.5 * FLUID_DENSITY * TETHER_DIAM * TETHER_DRAG_COEFFICIENT;

class OrcaDragPlugin : public ModelPlugin
{
private:

  physics::LinkPtr base_link_;

  // Drag force will be applied to the center_of_mass_ (body frame)
  ignition::math::Vector3d center_of_mass_ {0, 0, 0};

  // Tether drag will be applied to the tether attachment point (body frame)
  ignition::math::Vector3d tether_attach_ {0, 0, 0};

  // Drag constants (body frame)
  ignition::math::Vector3d linear_drag_ {ROV_LINEAR_DRAG_X, ROV_LINEAR_DRAG_Y, ROV_LINEAR_DRAG_Z};
  ignition::math::Vector3d angular_drag_ {ANGULAR_DRAG_X, ANGULAR_DRAG_Y, ANGULAR_DRAG_Z};
  double tether_drag_ {TETHER_DRAG};

  // Distance to surface
  double surface_ {10};

  event::ConnectionPtr update_connection_;

public:

  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    std::string link_name {"base_link"};

    std::cout << std::endl;
    std::cout << "ORCA DRAG PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default center of mass: " << center_of_mass_ << std::endl;
    std::cout << "Default tether attachment point: " << tether_attach_ << std::endl;
    std::cout << "Default linear drag: " << linear_drag_ << std::endl;
    std::cout << "Default angular drag: " << angular_drag_ << std::endl;
    std::cout << "Default tether drag: " << tether_drag_ << std::endl;
    std::cout << "Default surface: " << surface_ << std::endl;

    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    if (sdf->HasElement("link"))
    {
      sdf::ElementPtr linkElem = sdf->GetElement("link"); // Only one link is supported

      if (linkElem->HasAttribute("name"))
      {
        linkElem->GetAttribute("name")->Get(link_name);
        std::cout << "Link name: " << link_name << std::endl;
      }

      if (linkElem->HasElement("center_of_mass"))
      {
        center_of_mass_ = linkElem->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
        std::cout << "Center of mass: " << center_of_mass_ << std::endl;
      }

      if (linkElem->HasElement("tether_attach"))
      {
        tether_attach_ = linkElem->GetElement("tether_attach")->Get<ignition::math::Vector3d>();
        std::cout << "Tether attachment point: " << tether_attach_ << std::endl;
      }

      if (linkElem->HasElement("linear_drag"))
      {
        linear_drag_ = linkElem->GetElement("linear_drag")->Get<ignition::math::Vector3d>();
        std::cout << "Linear drag: " << linear_drag_ << std::endl;
      }

      if (linkElem->HasElement("angular_drag"))
      {
        angular_drag_ = linkElem->GetElement("angular_drag")->Get<ignition::math::Vector3d>();
        std::cout << "Angular drag: " << angular_drag_ << std::endl;
      }

      if (linkElem->HasElement("tether_drag")) // TODO should be child of gazebo element, not link element
      {
        tether_drag_ = linkElem->GetElement("tether_drag")->Get<double>();
        std::cout << "Tether drag: " << tether_drag_ << std::endl;
      }

      if (linkElem->HasElement("surface")) // TODO should be child of gazebo element, not link element
      {
        surface_ = linkElem->GetElement("surface")->Get<double>();
        std::cout << "Surface: " << surface_ << std::endl;
      }
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Listen for the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OrcaDragPlugin::OnUpdate, this, _1));

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the world update start event, up to 1000 times per second.
  void OnUpdate(const common::UpdateInfo& /*info*/)
  {
    ignition::math::Vector3d linear_velocity = base_link_->RelativeLinearVel();
    ignition::math::Vector3d angular_velocity = base_link_->RelativeAngularVel();

    ignition::math::Vector3d drag_force;
    drag_force.X() = linear_velocity.X() * fabs(linear_velocity.X()) * -linear_drag_.X();
    drag_force.Y() = linear_velocity.Y() * fabs(linear_velocity.Y()) * -linear_drag_.Y();
    drag_force.Z() = linear_velocity.Z() * fabs(linear_velocity.Z()) * -linear_drag_.Z();
    base_link_->AddLinkForce(drag_force, center_of_mass_);

    ignition::math::Vector3d drag_torque;
    drag_torque.X() = angular_velocity.X() * fabs(angular_velocity.X()) * -angular_drag_.X();
    drag_torque.Y() = angular_velocity.Y() * fabs(angular_velocity.Y()) * -angular_drag_.Y();
    drag_torque.Z() = angular_velocity.Z() * fabs(angular_velocity.Z()) * -angular_drag_.Z();
    base_link_->AddRelativeTorque(drag_torque); // ODE adds torque at the center of mass

    // Tether drag only accounts for motion in x (forward/reverse)
    ignition::math::Vector3d tether_force;
    double depth = surface_ - base_link_->WorldPose().Pos().Z();
    tether_force.X() = linear_velocity.X() * fabs(linear_velocity.X()) * depth * -tether_drag_;
    tether_force.Y() = 0;
    tether_force.Z() = 0;
    base_link_->AddLinkForce(tether_force, tether_attach_);
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaDragPlugin)

}
