#ifndef ORCA_MODEL_H
#define ORCA_MODEL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "orca_base/util.h"

namespace orca_base {

//=====================================================================================
// 4 DoF pose, in the world frame
// Everything is ENU, yaw == 0 when facing east
//=====================================================================================

struct OrcaPose
{
  double x_;
  double y_;
  double depth_; // TODO depth == -z, so not ENU
  double yaw_;

  OrcaPose(): x_(0), y_(0), depth_(0), yaw_(0) {}
  OrcaPose(double x, double y, double depth, double yaw): x_(x), y_(y), depth_(depth), yaw_(yaw) {}

  void toMsg(geometry_msgs::Pose &msg) const
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    msg.orientation = tf2::toMsg(q);

    msg.position.x = x_;
    msg.position.y = y_;
    msg.position.z = -depth_;
  }
};

// XY distance between 2 poses
constexpr double distance_xy(const OrcaPose &a, const OrcaPose &b)
{
  return std::hypot(a.x_ - b.x_, a.y_ - b.y_);
}

// Yaw distance between 2 poses
constexpr double distance_yaw(const OrcaPose &a, const OrcaPose &b)
{
  return std::abs(norm_angle(a.yaw_ - b.yaw_));
}

//=====================================================================================
// Thruster efforts from joystick or pid controllers, in the body frame
// Ranges from 1.0 for forward to -1.0 for reverse
//=====================================================================================

struct OrcaEfforts
{
  double forward_;
  double strafe_;
  double vertical_;
  double yaw_;

  OrcaEfforts(): forward_(0), strafe_(0), vertical_(0), yaw_(0) {}
  OrcaEfforts(double forward, double strafe, double depth, double yaw): forward_(forward), strafe_(strafe), vertical_(depth), yaw_(yaw) {}

  void clear() { forward_ = 0; strafe_ = 0; vertical_ = 0; yaw_ = 0; }
};

//=====================================================================================
// Orca description
// All vehicle specs are in the body frame (x forward, y left, z up)
// TODO move this to orca_description
// TODO pull overrides from the URDF file
// TODO consider matrices
//=====================================================================================

constexpr double FLUID_DENSITY = 1029;    // Fluid density of seawater

constexpr double ROV_DIM_X = 0.457;       // Length
constexpr double ROV_DIM_Y = 0.338;       // Width
constexpr double ROV_DIM_Z = 0.254;       // Height

constexpr double TETHER_DIAM = 0.008;

constexpr double ROV_AREA_X = ROV_DIM_Y * ROV_DIM_Z;  // Fore, aft area
constexpr double ROV_AREA_Y = ROV_DIM_X * ROV_DIM_Z;  // Top, bottom area
constexpr double ROV_AREA_Z = ROV_DIM_X * ROV_DIM_Y;  // Port, starboard area

constexpr double ORCA_MASS = 10;

// Assume a uniform distribution of mass in the vehicle box
constexpr double MOMENT_OF_INERTIA_YAW = ORCA_MASS / 12.0 * (ROV_DIM_X * ROV_DIM_X + ROV_DIM_Y * ROV_DIM_Y);

// From BlueRobotics specs, all forces are in Newtons
constexpr double BOLLARD_FORCE_XY = 137;
constexpr double BOLLARD_FORCE_Z = 88;
constexpr double T200_MAX_POS_FORCE = 50;
constexpr double T200_MAX_NEG_FORCE = 40;

// Estimate maximum yaw torque by looking at 4 thrusters (2 forward, 2 reverse), each mounted ~tangent to a circle with radius = 18cm
constexpr double MAX_TORQUE_YAW = 0.18 * 2.0 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

//=====================================================================================
// Drag calculations
//
// drag = 0.5 * density * area * velocity^2 * coefficient
//    the drag coefficient for a box is 1.0
//    the drag coefficient for an unfaired tether is 1.2
//
// The ROV constants below capture all but velocity:
//    constant = 0.5 * density * area * coefficient
//
// The tether constant below captures all but depth and velocity:
//    constant = 0.5 * density * width * coefficient
//=====================================================================================

constexpr double DRAG_COEFFICIENT_X = 0.8;    // Estimated
constexpr double DRAG_COEFFICIENT_Y = 0.95;   // Estimated
constexpr double DRAG_COEFFICIENT_Z = 0.95;   // Estimated

constexpr double LINEAR_DRAG_X = 0.5 * FLUID_DENSITY * ROV_AREA_X * DRAG_COEFFICIENT_X;
constexpr double LINEAR_DRAG_Y = 0.5 * FLUID_DENSITY * ROV_AREA_Y * DRAG_COEFFICIENT_Y;
constexpr double LINEAR_DRAG_Z = 0.5 * FLUID_DENSITY * ROV_AREA_Z * DRAG_COEFFICIENT_Z;
constexpr double LINEAR_DRAG_XY = 0.5 * (LINEAR_DRAG_X + LINEAR_DRAG_Y); // Average of x and y

constexpr double ANGULAR_DRAG_ROLL = 0.5 * LINEAR_DRAG_Z;   // Estimated
constexpr double ANGULAR_DRAG_PITCH = 0.5 * LINEAR_DRAG_Z;  // Estimated
constexpr double ANGULAR_DRAG_YAW = 0.5 * LINEAR_DRAG_XY;   // Estimated

constexpr double TETHER_DRAG_COEFFICIENT = 1.1;   // Estimated
constexpr double TETHER_DRAG = 0.5 * FLUID_DENSITY * TETHER_DIAM * TETHER_DRAG_COEFFICIENT;

//=====================================================================================
// Dynamics
// All force / torque calculations are done in the world frame
// Assume that roll and pitch are 0
//=====================================================================================

// Velocity => drag force / torque
constexpr double drag_force_x(double x_dot) { return x_dot * std::abs(x_dot) * -LINEAR_DRAG_X; }
constexpr double drag_force_y(double y_dot) { return y_dot * std::abs(y_dot) * -LINEAR_DRAG_Y; }
constexpr double drag_force_xy(double xy_dot) { return xy_dot * std::abs(xy_dot) * -LINEAR_DRAG_XY; }
constexpr double drag_force_z(double z_dot) { return z_dot * std::abs(z_dot) * -LINEAR_DRAG_Z; }
constexpr double drag_torque_yaw(double yaw_dot) { return yaw_dot * std::abs(yaw_dot) * -ANGULAR_DRAG_YAW; }

// Force / torque => acceleration
constexpr double force_to_accel_xy(double xy_force) { return xy_force / ORCA_MASS; }
constexpr double force_to_accel_z(double z_force) { return z_force / ORCA_MASS; } // TODO buoyancy and gravity
constexpr double torque_to_accel_yaw(double yaw_torque) { return yaw_torque / MOMENT_OF_INERTIA_YAW; }

// Force / torque => effort
// TODO deadband
constexpr double force_to_effort_xy(double force_xy) { return clamp(force_xy / BOLLARD_FORCE_XY, -1.0, 1.0); }
constexpr double force_to_effort_z(double force_z) { return clamp(force_z / BOLLARD_FORCE_Z, -1.0, 1.0); }
constexpr double torque_to_effort_yaw(double torque_yaw) { return clamp(torque_yaw / MAX_TORQUE_YAW, -1.0, 1.0); }

// Acceleration => force / torque
constexpr double accel_to_force_xy(double xy_dot_dot) { return ORCA_MASS * xy_dot_dot; }
constexpr double accel_to_force_z(double z_dot_dot) { return ORCA_MASS * z_dot_dot; }
constexpr double accel_to_torque_yaw(double yaw_dot_dot) { return MOMENT_OF_INERTIA_YAW * yaw_dot_dot; }

// Acceleration => effort
// TODO deadband
constexpr double accel_to_effort_xy(double xy_dot_dot) { return force_to_effort_xy(accel_to_force_xy(xy_dot_dot)); }
constexpr double accel_to_effort_z(double z_dot_dot) { return force_to_effort_z(accel_to_force_z(z_dot_dot)); }
constexpr double accel_to_effort_yaw(double yaw_dot_dot) { return torque_to_effort_yaw(accel_to_torque_yaw(yaw_dot_dot)); }

} // namespace orca_base

#endif // ORCA_MODEL_H