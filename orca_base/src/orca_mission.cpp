#include "orca_base/orca_mission.h"

namespace orca_base {

//=====================================================================================
// BaseMission
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

bool BaseMission::advance(const OrcaPose &curr, OrcaPose &plan, OrcaEfforts &efforts)
{
  efforts.clear();

  ros::Time now = ros::Time::now();
  dt_ = (now - last_time_).toSec();
  last_time_ = now;
}


//=====================================================================================
// SurfaceMission
//=====================================================================================

bool SurfaceMission::init(const OrcaPose &start, const OrcaPose &goal)
{
  double angle_to_goal = atan2(goal.y - start.y, goal.x - start.x);

  // Phase::turn
  goal1_ = start;
  goal1_.yaw = angle_to_goal;

  // Phase::run
  goal2_ = goal;
  goal2_.yaw = angle_to_goal;

  // Phase::final_turn
  goal3_ = goal;

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

bool SurfaceMission::advance(const OrcaPose &curr, OrcaPose &plan, OrcaEfforts &efforts)
{
  if (phase_ == Phase::no_goal)
  {
    return false;
  }

  // Update the plan
  if (!planner_->advance(dt_, curr, plan, efforts))
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
//=====================================================================================

constexpr bool is_rotate_phase(int p) { return p % 2 == 0; }

bool SquareMission::init(const OrcaPose &start, const OrcaPose &goal)
{
  constexpr double TARGET_DEPTH = -9;

  // Clear previous segments, if any
  segments_.clear();

  // Start
  segments_.push_back(Segment{Planner::start, start});

  if (std::abs(start.z - TARGET_DEPTH) > EPSILON_PLAN_XYZ)
  {
    // Descend/ascend to target depth
    segments_.push_back(Segment{Planner::vertical, OrcaPose(start.x, start.y, TARGET_DEPTH, start.yaw)});
  }

  bool north_first = goal.y > start.y;  // North or south first?
  bool east_first = goal.x > start.x;   // East or west first?

  // First leg
  segments_.push_back(Segment{Planner::rotate, OrcaPose(start.x, start.y, TARGET_DEPTH, north_first ? M_PI_2 : -M_PI_2)});
  segments_.push_back(Segment{Planner::line, OrcaPose(start.x, goal.y, TARGET_DEPTH, north_first ? M_PI_2 : -M_PI_2)});

  // Second leg
  segments_.push_back(Segment{Planner::rotate, OrcaPose(start.x, goal.y, TARGET_DEPTH, east_first ? 0 : M_PI)});
  segments_.push_back(Segment{Planner::line, OrcaPose(goal.x, goal.y, TARGET_DEPTH, east_first ? 0 : M_PI)});

  // Third leg
  segments_.push_back(Segment{Planner::rotate, OrcaPose(goal.x, goal.y, TARGET_DEPTH, north_first ? -M_PI_2 : M_PI_2)});
  segments_.push_back(Segment{Planner::line, OrcaPose(goal.x, start.y, TARGET_DEPTH, north_first ? -M_PI_2 : M_PI_2)});

  // Fourth leg
  segments_.push_back(Segment{Planner::rotate, OrcaPose(goal.x, start.y, TARGET_DEPTH, east_first ? M_PI : 0)});
  segments_.push_back(Segment{Planner::line, OrcaPose(start.x, start.y, TARGET_DEPTH, east_first ? M_PI : 0)});

  // Final turn
  segments_.push_back(Segment{Planner::rotate, OrcaPose(start.x, start.y, TARGET_DEPTH, goal.yaw)});

  // Start
  segment_ = 0;
  last_time_ = ros::Time::now();

  return true;
}

// TODO move to base class
bool SquareMission::advance(const OrcaPose &curr, OrcaPose &plan, OrcaEfforts &efforts)
{
  BaseMission::advance(curr, plan, efforts);

  // Update the plan
  if (!planner_ || !planner_->advance(dt_, curr, plan, efforts))
  {
    if (++segment_ >= segments_.size()) // Skips segment 0, which is the start segment
    {
      // Mission complete
      return false;
    }
    else
    {
      switch (segments_[segment_].planner)
      {
        case Planner::vertical:
          planner_.reset(new VerticalMotion);
          break;
        case Planner::rotate:
          planner_.reset(new RotateMotion);
          break;
        default:
          planner_.reset(new LineMotion);
          break;
      }

      if (!planner_->init(segments_[segment_ - 1].goal, segments_[segment_].goal))
      {
        ROS_ERROR("Can't init SquareMission phase %d", segment_);
        return false;
      }
    }
  }

  return true;
}

//=====================================================================================
// ArcMission
//=====================================================================================

bool ArcMission::init(const OrcaPose &start, const OrcaPose &goal)
{
  // Hold z constant
  OrcaPose goal2 = goal;
  goal2.z = start.z;

  planner_.reset(new ArcMotion);
  if (!planner_->init(start, goal2))
  {
    ROS_ERROR("Can't init ArcMission");
    return false;
  }

  last_time_ = ros::Time::now();
  return true;

}

bool ArcMission::advance(const OrcaPose &curr, OrcaPose &plan, OrcaEfforts &efforts)
{
  BaseMission::advance(curr, plan, efforts);
  return planner_->advance(dt_, curr, plan, efforts);
}

//=====================================================================================
// VerticalMission
//=====================================================================================

bool VerticalMission::init(const OrcaPose &start, const OrcaPose &goal)
{
  // Hold x, y, yaw constant
  OrcaPose goal2 = start;
  goal2.z = goal.z;

  planner_.reset(new VerticalMotion);
  if (!planner_->init(start, goal2))
  {
    ROS_ERROR("Can't init VerticalMission");
    return false;
  }

  last_time_ = ros::Time::now();
  return true;

}

bool VerticalMission::advance(const OrcaPose &curr, OrcaPose &plan, OrcaEfforts &efforts)
{
  BaseMission::advance(curr, plan, efforts);
  return planner_->advance(dt_, curr, plan, efforts);
}

} // namespace orca_base