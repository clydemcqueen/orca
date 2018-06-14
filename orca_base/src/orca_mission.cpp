#include "orca_base/orca_mission.h"

namespace orca_base {

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
  if (!planner_->advance(dt_, curr, plan_, efforts))
  {
    switch (phase_)
    {
      case Phase::turn:
        phase_ = Phase::run;
        planner_.reset(new LineMotion);
        if (!planner_->init(goal1_, goal2_))
        {
          ROS_ERROR("Can't init SurfaceMission Phase::run");
          return false;
        }
        break;

      case Phase::run:
        phase_ = Phase::final_turn;
        planner_.reset(new RotateMotion);
        if (!planner_->init(goal2_, goal3_))
        {
          ROS_ERROR("Can't init SurfaceMission Phase::final_turn");
          return false;
        }
        break;

      default:
        phase_ = Phase::no_goal;
        return false;
    }
  }

  return true;
}

//=====================================================================================
// SquareMission
//
// Run in a square defined by 2 points
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

constexpr bool is_rotate_phase(int p) { return p % 2 == 0; }

bool SquareMission::init(const OrcaPose &start, const OrcaPose &goal, nav_msgs::Path &path)
{
  BaseMission::init(start, goal, path);

  // Clear previous goals, if any
  goals_.clear();

  bool north_first = goal.y_ > start.y_;  // North or south first?
  bool east_first = goal.x_ > start.x_;   // East or west first?

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

  // Start phase 0
  phase_ = 0;
  planner_.reset(new RotateMotion);
  if (!planner_->init(start, goals_[0]))
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
  if (!planner_->advance(dt_, curr, plan_, efforts))
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
      // Start the next phase
      if (is_rotate_phase(phase_))
      {
        planner_.reset(new RotateMotion);
      }
      else
      {
        planner_.reset(new LineMotion);
      }

      if (!planner_->init(goals_[phase_ - 1], goals_[phase_]))
      {
        ROS_ERROR("Can't init SquareMission phase %d", phase_);
        return false;
      }
    }
  }

  return true;
}

} // namespace orca_base