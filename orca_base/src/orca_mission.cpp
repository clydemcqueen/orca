#include "orca_base/orca_mission.h"

namespace orca_base {

constexpr const double XY_VELO = 0.75;            // Cruising velocity for xy motion (m/s)
constexpr const double XY_EPSILON = 0.3;          // Close enough for xy motion (m)

constexpr const double YAW_VELO = M_PI / 5;       // Rotation velocity (r/s)
constexpr const double YAW_EPSILON = M_PI / 16;   // Close enough for rotation (r)

// Move an angle to the region [-M_PI, M_PI]
double norm_angle(double a)
{
  while (a < -M_PI)
  {
    a += 2 * M_PI;
  }
  while (a > M_PI)
  {
    a -= 2 * M_PI;
  }

  return a;
}

void BaseMotion::init(const MotionState &start, const MotionState &goal, MotionState &plan)
{
  goal_ = goal;
}

void RotateMotion::init(const MotionState &start, const MotionState &goal, MotionState &plan)
{
  BaseMotion::init(start, goal, plan);

  // Pick the shortest direction, assume instant acceleration
  plan.yaw_dot_ = norm_angle(goal.yaw_ - start.yaw_) > 0 ? YAW_VELO : -YAW_VELO;

  ROS_DEBUG("Rotate init: start angle %g, goal angle %g, plan yaw_dot_ %g", start.yaw_, goal.yaw_, plan.yaw_dot_);
}

bool RotateMotion::advance(double dt, MotionState &plan)
{
  if (std::abs(norm_angle(goal_.yaw_ - plan.yaw_)) > YAW_EPSILON)
  {
    plan.yaw_ = norm_angle(plan.yaw_ + plan.yaw_dot_ * dt);
    ROS_DEBUG("Rotate advance: goal %g, plan %g", goal_.yaw_, plan.yaw_);
    return true;
  }
  else
  {
    plan = goal_;
    return false;
  }
}

void LineMotion::init(const MotionState &start, const MotionState &goal, MotionState &plan)
{
  BaseMotion::init(start, goal, plan);

  double angle_to_goal = atan2(goal.y_ - start.y_, goal.x_ - start.x_);

  // Assume instant acceleration
  plan.x_dot_ = XY_VELO * cos(angle_to_goal);
  plan.y_dot_ = XY_VELO * sin(angle_to_goal);

  ROS_DEBUG("Line init: start (%g, %g), goal (%g, %g) angle to goal %g, plan x_dot %g, plan y_dot %g",
    start.x_, start.y_, goal.x_, goal.y_, angle_to_goal, plan.x_dot_, plan.y_dot_);
}

// Return true to continue, false if we're done
bool LineMotion::advance(double dt, MotionState &plan)
{
  if (std::hypot(goal_.x_ - plan.x_, goal_.y_ - plan.y_) > XY_EPSILON)
  {
    // Update our plan
    plan.x_ += plan.x_dot_ * dt;
    plan.y_ += plan.y_dot_ * dt;
    ROS_DEBUG("Line advance: goal x %g, goal y %g, plan x %g, plan y %g", goal_.x_, goal_.y_, plan.x_, plan.y_);
    return true;
  }
  else
  {
    plan = goal_;
    return false;
  }
}

void SurfaceMission::init(const MotionState &start, const MotionState &goal)
{
  double angle_to_goal = atan2(goal.y_ - start.y_, goal.x_ - start.x_);

  // Phase::turn
  goal1_ = start;
  goal1_.yaw_ = angle_to_goal;

  // Phase::run
  goal2_ = goal;
  goal2_.yaw_ = angle_to_goal;

  // Phase::final_turn
  goal3_ = goal;

  // Start Phase::turn
  last_time_ = ros::Time::now();
  phase_ = Phase::turn;
  predictor_.reset(new RotateMotion);
  predictor_->init(start, goal1_, plan_);

  // Ignore goal depth
  z_controller_.setTarget(UNDER_SURFACE);
}

bool SurfaceMission::advance(const MotionState &curr, OrcaEfforts &efforts)
{
  efforts.clear();

  if (phase_ == Phase::no_goal)
  {
    return false;
  }

  ros::Time now = ros::Time::now();
  dt_ = (now - last_time_).toSec();
  last_time_ = now;

  // Update the plan
  switch (phase_)
  {
    case Phase::turn:
      if (!predictor_->advance(dt_, plan_))
      {
        phase_ = Phase::run;
        predictor_.reset(new LineMotion);
        predictor_->init(goal1_, goal2_, plan_);
      }
      break;

    case Phase::run:
      if (!predictor_->advance(dt_, plan_))
      {
        phase_ = Phase::final_turn;
        predictor_.reset(new RotateMotion);
        predictor_->init(goal2_, goal3_, plan_);
      }
      break;

    default:
      if (!predictor_->advance(dt_, plan_))
      {
        phase_ = Phase::no_goal;
      }
      break;
  }

  // Set plan
  // TODO only set targets that need setting
  // TODO too much oscillation -- bouncing around -- why?
  x_controller_.setTarget(plan_.x_);
  y_controller_.setTarget(plan_.y_);
  yaw_controller_.setTarget(plan_.yaw_);

  // Calc desired x and y efforts
  double x_dot_dot = x_controller_.calc(curr.x_, dt_, 0);
  double y_dot_dot = y_controller_.calc(curr.y_, dt_, 0);

  // Rotate frame to get forward and strafe efforts
  efforts.forward_ = x_dot_dot * cos(curr.yaw_) + y_dot_dot * sin(curr.yaw_);
  efforts.strafe_ = y_dot_dot * cos(curr.yaw_) - x_dot_dot * sin(curr.yaw_);

  // Calc desired vertical and yaw efforts
  efforts.vertical_ = z_controller_.calc(curr.depth_, dt_, 0);
  efforts.yaw_ = yaw_controller_.calc(curr.yaw_, dt_, 0);

  // TODO pause 1s between phases?
  // TODO how do we handle "catch up"?

  return true;
}

} // namespace orca_base