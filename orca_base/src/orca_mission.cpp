#include "orca_base/orca_mission.h"

namespace orca_base {

constexpr const double XY_VELO = 0.75;            // Cruising velocity for xy motion (m/s)
constexpr const double XY_EPSILON = 0.3;          // Close enough for xy motion (m)

constexpr const double YAW_VELO = M_PI / 5;       // Rotation velocity (r/s)
constexpr const double YAW_EPSILON = M_PI / 16;   // Close enough for rotation (r)

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

//=====================================================================================
// Abstract base class for motion planning
//=====================================================================================

bool BaseMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  goal_ = goal;
}

//=====================================================================================
// Plan motion about a point
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

  ROS_DEBUG("Rotate init: start angle %g, goal angle %g, plan yaw_dot_ %g", start.yaw_, goal.yaw_, yaw_dot_);
  return true;
}

bool RotateMotion::advance(double dt, OrcaPose &plan)
{
  if (!close_enough_yaw(goal_, plan))
  {
    plan.yaw_ = norm_angle(plan.yaw_ + yaw_dot_ * dt);
    ROS_DEBUG("Rotate advance: goal %g, plan %g", goal_.yaw_, plan.yaw_);
    return true;
  }
  else
  {
    plan = goal_;
    return false;
  }
}

//=====================================================================================
// Plan motion from point A to point B
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

  ROS_DEBUG("Line init: start (%g, %g), goal (%g, %g) angle to goal %g, plan x_dot %g, plan y_dot %g",
    start.x_, start.y_, goal.x_, goal.y_, angle_to_goal, x_dot_, y_dot_);
  return true;
}

bool LineMotion::advance(double dt, OrcaPose &plan)
{
  if (!close_enough_xy(goal_, plan))
  {
    // Update our plan
    plan.x_ += x_dot_ * dt;
    plan.y_ += y_dot_ * dt;
    ROS_DEBUG("Line advance: goal x %g, goal y %g, plan x %g, plan y %g", goal_.x_, goal_.y_, plan.x_, plan.y_);
    return true;
  }
  else
  {
    plan = goal_;
    return false;
  }
}

//=====================================================================================
// Abstract base class for running missions
//=====================================================================================

void BaseMission::addToPath(nav_msgs::Path &path, const OrcaPose &pose)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp =  path.header.stamp; // TODO use predicted/actual time
  msg.header.frame_id = path.header.frame_id;
  pose.toMsg(msg.pose);
  path.poses.push_back(msg);
}

void BaseMission::addToPath(nav_msgs::Path &path, const std::vector<OrcaPose> &poses)
{
  for (int i = 0; i < poses.size(); ++i)
  {
    addToPath(path, poses[i]);
  }
}

bool BaseMission::init(const OrcaPose &start, const OrcaPose &goal, nav_msgs::Path &path)
{
  // Init plan
  plan_ = start;

  // Init path message
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  addToPath(path, start);

  // Ignore goal depth
  z_controller_.setTarget(UNDER_SURFACE);
}

bool BaseMission::advance(const OrcaPose &curr, OrcaEfforts &efforts)
{
  efforts.clear();

  ros::Time now = ros::Time::now();
  dt_ = (now - last_time_).toSec();
  last_time_ = now;
}

//=====================================================================================
// SurfaceMission
//
// Run from point A to B at the surface
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

bool SurfaceMission::init(const OrcaPose &start, const OrcaPose &goal, nav_msgs::Path &path)
{
  BaseMission::init(start, goal, path);

  double angle_to_goal = atan2(goal.y_ - start.y_, goal.x_ - start.x_);

  // Phase::turn
  goal1_ = start;
  goal1_.yaw_ = angle_to_goal;
  addToPath(path, goal1_);

  // Phase::run
  goal2_ = goal;
  goal2_.yaw_ = angle_to_goal;
  addToPath(path, goal2_);

  // Phase::final_turn
  goal3_ = goal;
  addToPath(path, goal3_);

  // Start Phase::turn
  phase_ = Phase::turn;
  planner_.reset(new RotateMotion);
  if (!planner_->init(start, goal1_))
  {
    ROS_ERROR("Can't init SurfaceMission Phase::turn");
    return false;
  }

  last_time_ = ros::Time::now();
  return true;
}

bool SurfaceMission::advance(const OrcaPose &curr, OrcaEfforts &efforts)
{
  BaseMission::advance(curr, efforts);

  if (phase_ == Phase::no_goal)
  {
    return false;
  }

  // Update the plan
  switch (phase_)
  {
    case Phase::turn:
      if (!planner_->advance(dt_, plan_))
      {
        phase_ = Phase::run;
        planner_.reset(new LineMotion);
        if (!planner_->init(goal1_, goal2_))
        {
          ROS_ERROR("Can't init SurfaceMission Phase::run");
          return false;
        }
      }
      break;

    case Phase::run:
      if (!planner_->advance(dt_, plan_))
      {
        phase_ = Phase::final_turn;
        planner_.reset(new RotateMotion);
        if (!planner_->init(goal2_, goal3_))
        {
          ROS_ERROR("Can't init SurfaceMission Phase::final_turn");
          return false;
        }
      }
      break;

    default:
      if (!planner_->advance(dt_, plan_))
      {
        phase_ = Phase::no_goal;
        return false;
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

//=====================================================================================
// SquareMission
//
// Run in a square defined by 2 points
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

bool SquareMission::init(const OrcaPose &start, const OrcaPose &goal, nav_msgs::Path &path)
{
  BaseMission::init(start, goal, path);

  // Clear previous goals
  goals_.clear();

  bool north_first = goal.y_ > start.y_;
  bool east_first = goal.x_ > start.x_;

  // First leg
  goals_.push_back(OrcaPose(start.x_, start.y_, UNDER_SURFACE, north_first ? M_PI_2 : -M_PI_2)); // Turn
  goals_.push_back(OrcaPose(start.x_, goal.y_, UNDER_SURFACE, north_first ? M_PI_2 : -M_PI_2)); // Move

  // Second leg
  goals_.push_back(OrcaPose(start.x_, goal.y_, UNDER_SURFACE, east_first ? 0 : M_PI)); // Turn
  goals_.push_back(OrcaPose(goal.x_, goal.y_, UNDER_SURFACE, east_first ? 0 : M_PI)); // Move

  // Third leg
  goals_.push_back(OrcaPose(goal.x_, goal.y_, UNDER_SURFACE, north_first ? -M_PI_2 : M_PI_2)); // Turn
  goals_.push_back(OrcaPose(goal.x_, start.y_, UNDER_SURFACE, north_first ? -M_PI_2 : M_PI_2)); // Move

  // Fourth leg
  goals_.push_back(OrcaPose(goal.x_, start.y_, UNDER_SURFACE, east_first ? M_PI : 0)); // Turn
  goals_.push_back(OrcaPose(start.x_, start.y_, UNDER_SURFACE, east_first ? M_PI : 0)); // Move

  // Final turn
  goals_.push_back(OrcaPose(start.x_, start.y_, UNDER_SURFACE, goal.yaw_)); // Turn

  // Add goals to the path
  addToPath(path, goals_);

  // Start
  phase_ = 0;
  planner_.reset(new RotateMotion);
  if (!planner_->init(start, goals_[phase_]))
  {
    ROS_ERROR("Can't init SquareMission phase 1");
    return false;
  }

  last_time_ = ros::Time::now();
  return true;
}

bool SquareMission::advance(const OrcaPose &curr, OrcaEfforts &efforts)
{
  BaseMission::advance(curr, efforts);

  if (phase_ == NO_GOAL)
  {
    return false;
  }

  // Update the plan
  if (!planner_->advance(dt_, plan_))
  {
    // That phase is done
    if (++phase_ >= goals_.size())
    {
      // Mission is complete
      phase_ = NO_GOAL;
      return false;
    }
    else
    {
      // Create a planner for the next phase
      if (phase_ % 2 == 0)
      {
        planner_.reset(new RotateMotion);
      }
      else
      {
        planner_.reset(new LineMotion);
      }

      // Init the next phase
      if (!planner_->init(goals_[phase_ - 1], goals_[phase_]))
      {
        ROS_ERROR("Can't init SquareMission phase %d", phase_);
        return false;
      }
    }
  }

  // Set plan
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

  return true;
}

} // namespace orca_base