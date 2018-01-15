#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

/* A simple drag plugin. Usage:
 *
 *    <gazebo>
 *      <plugin name="DragPlugin" filename="libDragPlugin.so">
 *        <link name="base_link">
 *          <center_of_mass>0 0 -0.2</center_of_mass>
 *          <linear_drag>10 20 30</linear_drag>
 *          <angular_drag>5 10 15</angular_drag>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <center_of_mass> Drag force is applied to the center of mass.
 *    <linear_drag> Linear drag constants. See default calculation.
 *    <angular_drag> Angular drag constants. See defaut calculation.
 *
 *    This plugin does not model tether drag.
 */

namespace gazebo {

// Default calculation for generating drag constants for a rectangular ROV:
constexpr double DIM_X = 0.457;           // Length
constexpr double DIM_Y = 0.338;           // Width
constexpr double DIM_Z = 0.254;           // Height
constexpr double AREA_X = DIM_Y * DIM_Z;  // Fore, aft area
constexpr double AREA_Y = DIM_X * DIM_Z;  // Top, bottom area
constexpr double AREA_Z = DIM_X * DIM_Y;  // Port, starboard area
constexpr double FLUID_DENSITY = 1029;    // Fluid density of seawater
constexpr double DRAG_COEFFICIENT = 0.9;  // Default drag coefficient for a flat surface
constexpr double LINEAR_DRAG_X = 0.5 * FLUID_DENSITY * AREA_X * DRAG_COEFFICIENT;
constexpr double LINEAR_DRAG_Y = 0.5 * FLUID_DENSITY * AREA_Y * DRAG_COEFFICIENT;
constexpr double LINEAR_DRAG_Z = 0.5 * FLUID_DENSITY * AREA_Z * DRAG_COEFFICIENT;
constexpr double ANGULAR_DRAG_X = LINEAR_DRAG_X / 2; // A hack
constexpr double ANGULAR_DRAG_Y = LINEAR_DRAG_Y / 2;
constexpr double ANGULAR_DRAG_Z = LINEAR_DRAG_Z / 2;

class DragPlugin : public ModelPlugin
{
private:

  // Drag force will be applied to the center_of_mass_ of base_link_
  physics::LinkPtr base_link_;
  math::Vector3 center_of_mass_ {0, 0, 0};

  // Drag constants
  math::Vector3 linear_drag_ {LINEAR_DRAG_X, LINEAR_DRAG_Y, LINEAR_DRAG_Z};
  math::Vector3 angular_drag_ {ANGULAR_DRAG_X, ANGULAR_DRAG_Y, ANGULAR_DRAG_Z};

  event::ConnectionPtr update_connection_;

public:

  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    std::string link_name {"base_link"};

    std::cout << std::endl;
    std::cout << "DRAG PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default center of gravity: " << center_of_mass_ << std::endl;
    std::cout << "Default linear drag: " << linear_drag_ << std::endl;
    std::cout << "Default angular drag: " << angular_drag_ << std::endl;

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
        center_of_mass_ = linkElem->GetElement("center_of_mass")->Get<math::Vector3>();
        std::cout << "Center of gravity: " << center_of_mass_ << std::endl;
      }

      if (linkElem->HasElement("linear_drag"))
      {
        linear_drag_ = linkElem->GetElement("linear_drag")->Get<math::Vector3>();
        std::cout << "Linear drag: " << linear_drag_ << std::endl;
      }

      if (linkElem->HasElement("angular_drag"))
      {
        angular_drag_ = linkElem->GetElement("angular_drag")->Get<math::Vector3>();
        std::cout << "Angular drag: " << angular_drag_ << std::endl;
      }
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Listen for the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DragPlugin::OnUpdate, this, _1));

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the world update start event, up to 1000 times per second.
  void OnUpdate(const common::UpdateInfo& /*info*/)
  {
    math::Vector3 linear_velocity = base_link_->GetRelativeLinearVel();
    math::Vector3 angular_velocity = base_link_->GetRelativeAngularVel();

    math::Vector3 drag_force;
    drag_force.x = linear_velocity.x * fabs(linear_velocity.x) * -linear_drag_.x;
    drag_force.y = linear_velocity.y * fabs(linear_velocity.y) * -linear_drag_.y;
    drag_force.z = linear_velocity.z * fabs(linear_velocity.z) * -linear_drag_.z;
    base_link_->AddLinkForce(drag_force, center_of_mass_);

    math::Vector3 drag_torque;
    drag_torque.x = angular_velocity.x * fabs(angular_velocity.x) * -angular_drag_.x;
    drag_torque.y = angular_velocity.y * fabs(angular_velocity.y) * -angular_drag_.y;
    drag_torque.z = angular_velocity.z * fabs(angular_velocity.z) * -angular_drag_.z;
    base_link_->AddRelativeTorque(drag_torque); // ODE adds torque at the center of mass
  }
};

GZ_REGISTER_MODEL_PLUGIN(DragPlugin)

}
