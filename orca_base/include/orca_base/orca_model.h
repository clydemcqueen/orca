#ifndef ORCA_MODEL_H
#define ORCA_MODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "orca_base/util.h"

namespace orca_base {

//=====================================================================================
// Vehicle specs, in the body frame (x forward, y left, z up)
//=====================================================================================

constexpr double GRAVITY = 9.80665;
constexpr double FLUID_DENSITY = 1029;    // Fluid density of seawater

constexpr double ROV_DIM_X = 0.457;       // Length
constexpr double ROV_DIM_Y = 0.338;       // Width
constexpr double ROV_DIM_Z = 0.254;       // Height

constexpr double TETHER_DIAM = 0.008;

constexpr double ROV_AREA_X = ROV_DIM_Y * ROV_DIM_Z;  // Area of front (bow) and rear (stern) sides
constexpr double ROV_AREA_Y = ROV_DIM_X * ROV_DIM_Z;  // Area of top and bottom
constexpr double ROV_AREA_Z = ROV_DIM_X * ROV_DIM_Y;  // Area of left (port) and right (starboard) sides

constexpr double ORCA_MASS = 10;
constexpr double ORCA_VOLUME = 0.01;

// Assume a uniform distribution of mass in the vehicle box
constexpr double MOMENT_OF_INERTIA_YAW = ORCA_MASS / 12.0 * (ROV_DIM_X * ROV_DIM_X + ROV_DIM_Y * ROV_DIM_Y);

// From BlueRobotics specs, all forces are in Newtons
constexpr double BOLLARD_FORCE_XY = 137;
constexpr double BOLLARD_FORCE_Z = 88;
constexpr double T200_MAX_POS_FORCE = 50;
constexpr double T200_MAX_NEG_FORCE = 40;

// Estimate maximum yaw torque by looking at 4 thrusters (2 forward, 2 reverse), each mounted ~tangent to a circle with radius = 18cm
constexpr double MAX_TORQUE_YAW = 0.18 * 2.0 * (T200_MAX_POS_FORCE + T200_MAX_NEG_FORCE);

constexpr double DISPLACED_MASS = ORCA_VOLUME * FLUID_DENSITY;
constexpr double WEIGHT_IN_WATER = GRAVITY * (ORCA_MASS - DISPLACED_MASS);
constexpr double HOVER_ACCEL_Z = WEIGHT_IN_WATER / ORCA_MASS; // Z acceleration required to hover

//=====================================================================================
// Drag constants, in the body frame (x forward, y left, z up)
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

constexpr double ANGULAR_DRAG_YAW = 0.5 * (LINEAR_DRAG_X + LINEAR_DRAG_Y);   // Estimated

constexpr double TETHER_DRAG_COEFFICIENT = 1.1;   // Estimated
constexpr double TETHER_DRAG = 0.5 * FLUID_DENSITY * TETHER_DIAM * TETHER_DRAG_COEFFICIENT;

//=====================================================================================
// Dynamics, in the world frame (ENU)
// 4 DoF, pitch and roll are 0
//=====================================================================================

// Velocity => drag force / torque
constexpr double drag_force_x(double velo_x) { return velo_x * std::abs(velo_x) * -LINEAR_DRAG_X; }
constexpr double drag_force_y(double velo_y) { return velo_y * std::abs(velo_y) * -LINEAR_DRAG_Y; }
constexpr double drag_force_z(double velo_z) { return velo_z * std::abs(velo_z) * -LINEAR_DRAG_Z; }
constexpr double drag_torque_yaw(double velo_yaw) { return velo_yaw * std::abs(velo_yaw) * -ANGULAR_DRAG_YAW; }

// Force / torque => acceleration
constexpr double force_to_accel_xy(double force_xy) { return force_xy / ORCA_MASS; }
constexpr double force_to_accel_z(double force_z) { return force_z / ORCA_MASS; } // TODO combine _xy and _z forms
constexpr double torque_to_accel_yaw(double torque_yaw) { return torque_yaw / MOMENT_OF_INERTIA_YAW; }

// Force / torque => effort
// TODO add deadband
constexpr double force_to_effort_xy(double force_xy) { return clamp(force_xy / BOLLARD_FORCE_XY, -1.0, 1.0); }
constexpr double force_to_effort_z(double force_z) { return clamp(force_z / BOLLARD_FORCE_Z, -1.0, 1.0); }
constexpr double torque_to_effort_yaw(double torque_yaw) { return clamp(torque_yaw / MAX_TORQUE_YAW, -1.0, 1.0); }

// Acceleration => force / torque
constexpr double accel_to_force_xy(double accel_xy) { return ORCA_MASS * accel_xy; }
constexpr double accel_to_force_z(double accel_z) { return ORCA_MASS * accel_z; }
constexpr double accel_to_torque_yaw(double accel_yaw) { return MOMENT_OF_INERTIA_YAW * accel_yaw; }

// Acceleration => effort
// TODO add deadband
constexpr double accel_to_effort_xy(double accel_xy) { return force_to_effort_xy(accel_to_force_xy(accel_xy)); }
constexpr double accel_to_effort_z(double accel_z) { return force_to_effort_z(accel_to_force_z(accel_z)); }
constexpr double accel_to_effort_yaw(double accel_yaw) { return torque_to_effort_yaw(accel_to_torque_yaw(accel_yaw)); }

//=====================================================================================
// 4 DoF pose, in the world frame
// ENU, yaw == 0 when facing east, z == 0 at the surface
//=====================================================================================

struct OrcaPose
{
  double x;
  double y;
  double z;
  double yaw;

  constexpr OrcaPose(): x(0), y(0), z(0), yaw(0) {}
  constexpr OrcaPose(double _x, double _y, double _z, double _yaw): x(_x), y(_y), z(_z), yaw(_yaw) {}
  OrcaPose(const tf2::Vector3 &vector, double _yaw): x(vector.x()), y(vector.y()), z(vector.z()), yaw(_yaw) {}

  void clear() { x = 0; y = 0; z = 0; yaw = 0; }

  void toMsg(geometry_msgs::Pose &msg) const
  {
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;

    // Yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg.orientation = tf2::toMsg(q);
  }

  void fromMsg(const nav_msgs::Odometry &msg)
  {
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;

    // Quaternion to yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg.pose.pose.orientation, q);
    double roll = 0, pitch = 0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  void fromMsg(const geometry_msgs::PoseStamped &msg)
  {
    x = msg.pose.position.x;
    y = msg.pose.position.y;
    z = msg.pose.position.z;

    // Quaternion to yaw TODO util
    tf2::Quaternion q;
    tf2::fromMsg(msg.pose.orientation, q);
    double roll = 0, pitch = 0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  // Distance between 2 poses on the xy plane
  double distance_xy(const OrcaPose &that) const
  {
    return std::hypot(x - that.x, y - that.y);
  }

  // Z distance between 2 poses
  double distance_z(const OrcaPose &that) const
  {
    return std::abs(z - that.z);
  }

  // Yaw distance between 2 poses
  double distance_yaw(const OrcaPose &that) const
  {
    return std::abs(norm_angle(yaw - that.yaw));
  }
};

//=====================================================================================
// 4 DoF odometry, in the world frame
//=====================================================================================

constexpr double DEF_COVAR = 0.05;

struct OrcaOdometry
{
  OrcaPose pose;
  std::array<double, 16> pose_covariance;

  OrcaPose velo;
  std::array<double, 16> velo_covariance;

  OrcaOdometry()
  {
    pose_covariance.fill(0);
    velo_covariance.fill(0);

    for (int i = 0; i < 4; ++i)
    {
      pose_covariance[i + 4 * i] = DEF_COVAR;
      velo_covariance[i + 4 * i] = DEF_COVAR;
    }
  }

  void stopMotion()
  {
    velo = OrcaPose{};
  }

  // Create a 6x6 covariance matrix (x, y, z, roll, pitch, yaw) from a 4x4 matrix (x, y, z, yaw)
  static void covar4to6(const std::array<double, 16> &four, boost::array<double, 36> &six)
  {
    six.fill(0);

    six[21] = DEF_COVAR; // roll diag
    six[28] = DEF_COVAR; // pitch diag

    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        six[(i < 3 ? i : i + 2) * 6 + (j < 3 ? j : j + 2)] = four[i * 4 + j];
      }
    }
  }

  void toMsg(nav_msgs::Odometry &msg) const
  {
    pose.toMsg(msg.pose.pose);

    msg.twist.twist.linear.x = velo.x;
    msg.twist.twist.linear.y = velo.y;
    msg.twist.twist.linear.z = velo.z;

    msg.twist.twist.angular.x = 0;
    msg.twist.twist.angular.y = 0;
    msg.twist.twist.angular.z = velo.yaw;

    covar4to6(pose_covariance, msg.pose.covariance);
    covar4to6(velo_covariance, msg.twist.covariance);
  }
};

//=====================================================================================
// Thruster efforts from joystick or pid controllers, in the body frame
// Ranges from 1.0 for forward to -1.0 for reverse
//=====================================================================================

struct OrcaEfforts
{
  double forward;
  double strafe;
  double vertical;
  double yaw;

  OrcaEfforts(): forward(0), strafe(0), vertical(0), yaw(0) {}

  void clear() { forward = 0; strafe = 0; vertical = 0; yaw = 0; }

  void fromAcceleration(const OrcaPose &u_bar, const double current_yaw)
  {
    // u_bar (acceleration) => u (control inputs normalized from -1 to 1, aka effort)
    double x_effort = accel_to_effort_xy(u_bar.x);
    double y_effort = accel_to_effort_xy(u_bar.y);
    double z_effort = accel_to_effort_z(u_bar.z);
    yaw = accel_to_effort_yaw(u_bar.yaw);

    // Convert from world frame to body frame
    vertical = -z_effort;
    rotate_frame(x_effort, y_effort, current_yaw, forward, strafe);
  }

  // TODO toAcceleration will convert joystick inputs => acceleration
};

} // namespace orca_base

#endif // ORCA_MODEL_H