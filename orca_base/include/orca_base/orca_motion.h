#ifndef ORCA_MOTION_H
#define ORCA_MOTION_H

#include <eigen3/Eigen/Geometry>
#include "orca_base/orca_model.h"
#include "orca_base/pid.h"

namespace orca_base {

//=====================================================================================
// Constants
//=====================================================================================

constexpr double UNDER_SURFACE_Z = -0.5;        // Cruise just below the surface

//=====================================================================================
// BaseMotion is a never-ending pid controller
// -- hold x, y, z, yaw at start value
//=====================================================================================

class BaseMotion
{
protected:

  // Goal state
  OrcaPose goal_;

  // Planned motion, updated at every timestep
  OrcaPose pose_;
  OrcaPose velo_;

  // Feedforward = planned acceleration + acceleration due to drag
  OrcaPose ff_;

  // PID controllers
  pid::Controller x_controller_{false, 0.5, 0.2};
  pid::Controller y_controller_{false, 0.5, 0.2};
  pid::Controller z_controller_{false, 0.5, 0.2};
  pid::Controller yaw_controller_{true, 0.5, 0.5};

public:

  // Initialize the motion plan, return true if successful
  virtual bool init(const OrcaPose &start, const OrcaPose &goal);

  // Advance the motion plan, return true to continue, false if we're done
  virtual bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts);
};

//=====================================================================================
// RotateMotion rotates about a point
// -- hold x, y, z at start value
//
// Limitations:
// -- assume instant thrust force
// -- assume instant acceleration
//=====================================================================================

class RotateMotion: public BaseMotion
{
public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts) override;
};

//=====================================================================================
// LineMotion moves in a straight line
// -- hold z, yaw at start value
//
// Phase 1: apply a constant thrust; asymptotically hit desired velocity
// Phase 2: cut thrusters and decelerate
//
// Limitations:
// -- assume instant thrust force
//=====================================================================================

class LineMotion: public BaseMotion
{
public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts) override;
};

//=====================================================================================
// ArcMotion moves in an arc
// -- radius is computed as the distance from start to goal
// -- yaw will always point in the direction of motion
// -- hold z at start value
//
// Limitations:
// -- center is always to the left of the start position, always moves ccw
// -- assume instant thrust force
// -- assume instant acceleration
//=====================================================================================

class ArcMotion: public BaseMotion
{
private:

  // Simple arc
  struct Arc
  {
    // Simple 2d point
    struct Point
    {
      double x;
      double y;
    };

    // ENU polar angle, 0 = East, increment counter-clockwise, in radians
    typedef double Polar;

    Point center_;
    double radius_;
    Polar start_angle_;   // Arc start angle
    Polar goal_angle_;    // Arc goal angle

    // Convert a polar angle on this arc to an OrcaPose
    constexpr void polarToCartesian(const Polar theta, OrcaPose &pose) const
    {
      pose.x = center_.x + radius_ * cos(theta);
      pose.y = center_.y + radius_ * sin(theta);
      pose.yaw = norm_angle(theta + M_PI_2);
    }

    // Convert an OrcaPose to a polar angle on this arc
    constexpr void cartesianToPolar(const OrcaPose &pose, Polar &theta) const
    {
      // TODO assert that pose is on the circle
      theta = atan2(pose.y - center_.y, pose.x - center_.x);
    }
  };

  // Planned motion in polar coordinates
  Arc arc_;
  Arc::Polar polar_pose_;
  Arc::Polar polar_velo_;

public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts) override;
};

//=====================================================================================
// VerticalMotion ascends or descends
// -- hold x, y, yaw at start value
//
// Limitations:
// -- assume instant thrust force
// -- assume instant acceleration
//=====================================================================================

class VerticalMotion: public BaseMotion
{
public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts) override;
};

} // namespace orca_base

#endif // ORCA_MOTION_H