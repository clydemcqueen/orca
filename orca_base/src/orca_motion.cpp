#include "orca_base/orca_motion.h"

namespace orca_base {

constexpr double XY_VELO = 0.5;             // Cruising velocity for xy motion (m/s)
constexpr double XY_EPSILON = 0.1;          // Close enough for xy motion (m), keep in mind that OrcaBase spins at 10Hz

constexpr double YAW_VELO = M_PI / 10;      // Rotation velocity (r/s)
constexpr double YAW_EPSILON = M_PI / 50;   // Close enough for yaw motion (r)

//=====================================================================================
// Utilities
//=====================================================================================

// Return true if we're x and y are close enough
constexpr bool close_enough_xy(const OrcaPose &a, const OrcaPose &b)
{
  return distance_xy(a, b) < XY_EPSILON;
}

// Return true if the yaw angle is close enough
constexpr bool close_enough_yaw(const OrcaPose &a, const OrcaPose &b)
{
  return distance_yaw(a, b) < YAW_EPSILON;
}

// Compute a 2d point in a rotated frame (v' = R_transpose * v)
void rotate_frame(const double x, const double y, const double theta, double &x_r, double &y_r)
{
  x_r = x * cos(theta) + y * sin(theta);
  y_r = y * cos(theta) - x * sin(theta);
}

//=====================================================================================
// Base motion is a never-ending pid controller
//=====================================================================================

bool BaseMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  goal_ = goal;

  // Init all pid controllers
  x_controller_.setTarget(goal.x_);
  y_controller_.setTarget(goal.y_);
  z_controller_.setTarget(UNDER_SURFACE); // Ignore depth for now
  yaw_controller_.setTarget(goal.yaw_);

  // Always succeed
  return true;
}

bool BaseMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  // Run the plan through the pid controllers to get u_bar (desired acceleration)
  double u_bar_x = x_controller_.calc(curr.x_, dt, x_dot_dot_);
  double u_bar_y = y_controller_.calc(curr.y_, dt, y_dot_dot_);
  double u_bar_z = z_controller_.calc(curr.depth_, dt, 0); // No feedforward for now
  double u_bar_yaw = yaw_controller_.calc(curr.yaw_, dt, yaw_dot_dot_);

  // Acceleration => effort (u, normalized from -1 to 1)
  double x_effort = accel_to_effort_xy(u_bar_x);
  double y_effort = accel_to_effort_xy(u_bar_y);
  efforts.yaw_ = accel_to_effort_yaw(u_bar_yaw);
  efforts.vertical_ = accel_to_effort_z(u_bar_z);

  // Rotate frame to get forward and strafe efforts
  rotate_frame(x_effort, y_effort, curr.yaw_, efforts.forward_, efforts.strafe_);

  // Never stop
  return true;
}

//=====================================================================================
// Rotate about a point
//=====================================================================================

bool RotateMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  BaseMotion::init(start, goal);

  if (!close_enough_xy(start, goal))
  {
    ROS_ERROR("Can't init rotate motion: start and goal positions are different");
    return false;
  }

  // Pick the shortest direction, assume instant acceleration
  yaw_dot_ = norm_angle(goal.yaw_ - start.yaw_) > 0 ? YAW_VELO : -YAW_VELO;

  // Drag force => thrust force => feed forward for the pid controller
  yaw_dot_dot_ = torque_to_accel_yaw(-drag_torque_yaw(yaw_dot_));

  ROS_DEBUG("Rotate init: start %g, goal %g, dot %g, dot_dot %g",
    start.yaw_, goal.yaw_, yaw_dot_, yaw_dot_dot_);
  return true;
}

bool RotateMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  if (!close_enough_yaw(goal_, plan))
  {
    // Update the plan
    plan.yaw_ = norm_angle(plan.yaw_ + yaw_dot_ * dt);
    yaw_controller_.setTarget(plan.yaw_);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan = goal_;
    return false;
  }
}

//=====================================================================================
// Move from point A to point B
//=====================================================================================

bool LineMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  BaseMotion::init(start, goal);

  if (!close_enough_yaw(start, goal))
  {
    ROS_ERROR("Can't init line motion: start and goal headings are different");
    return false;
  }

  double angle_to_goal = atan2(goal.y_ - start.y_, goal.x_ - start.x_);

  // Assume instant acceleration
  x_dot_ = XY_VELO * cos(angle_to_goal);
  y_dot_ = XY_VELO * sin(angle_to_goal);

  // Drag force => thrust force => feed forward for the pid controller
  // Rotate x and y into the body frame to calc drag, then rotate back to world frame
  double forward_dot, strafe_dot, forward_dot_dot, strafe_dot_dot;
  rotate_frame(x_dot_, y_dot_, goal.yaw_, forward_dot, strafe_dot);
  forward_dot_dot = force_to_accel_xy(-drag_force_x(forward_dot));
  strafe_dot_dot = force_to_accel_xy(-drag_force_y(strafe_dot));
  rotate_frame(forward_dot_dot, strafe_dot_dot, -goal.yaw_, x_dot_dot_, y_dot_dot_);

  ROS_DEBUG("Line init: start (%g, %g), goal (%g, %g), dot (%g, %g), dot_dot (%g, %g)",
    start.x_, start.y_, goal.x_, goal.y_, x_dot_, y_dot_, x_dot_dot_, y_dot_dot_);
  return true;
}

bool LineMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  if (!close_enough_xy(goal_, plan))
  {
    // Update the plan
    plan.x_ += x_dot_ * dt;
    plan.y_ += y_dot_ * dt;
    x_controller_.setTarget(plan.x_);
    y_controller_.setTarget(plan.y_);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan = goal_;
    return false;
  }
}

} // namespace orca_base