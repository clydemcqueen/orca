#include "orca_base/orca_mission.h"

namespace orca_base {

constexpr const double XY_VELO = 0.75;            // Cruising velocity for xy motion (m/s)
constexpr const double XY_EPSILON = 0.3;          // Close enough for xy motion (m)

constexpr const double YAW_VELO = M_PI / 5;       // Rotation velocity (r/s)
constexpr const double YAW_EPSILON = M_PI / 16;   // Close enough for rotation (r)

void BaseMotion::init(const MotionState &curr, const MotionState &goal)
{
  plan_ = curr;
  goal_ = goal;

  // All motion models run at a constant depth TODO this will change
  z_controller_.setTarget(goal.depth_);

  last_time_ = ros::Time::now();
};

bool BaseMotion::advance(const MotionState &curr, OrcaEfforts &efforts)
{
  // TODO
}

void RotateMotion::init(const MotionState &curr, const MotionState &goal)
{
  BaseMotion::init(curr, goal);

  // Assume instant acceleration
  plan_yaw_dot_ = YAW_VELO; // TODO pick the shortest direction

  ROS_DEBUG("Rotate 1. curr angle %g, goal angle %g, plan_yaw_dot_ %g", curr.yaw_, goal.yaw_, plan_yaw_dot_);
}

bool RotateMotion::advance(const MotionState &curr, OrcaEfforts &efforts)
{
  efforts.clear();

  if (std::abs(goal_.yaw_ - curr.yaw_) > YAW_EPSILON) // TODO discontinuity
  {
    double dt = (ros::Time::now() - last_time_).toSec();
    last_time_ = ros::Time::now();

    if (std::abs(goal_.yaw_ - plan_.yaw_) > YAW_EPSILON) // TODO discontinuity
    {
      // Update our plan
      plan_.yaw_ += plan_yaw_dot_ * dt;

      yaw_controller_.setTarget(plan_.yaw_);

      ROS_DEBUG("Rotate 2. goal %g, plan %g", goal_.yaw_, plan_.yaw_);
    }

    // Calc desired accelerations
    efforts.yaw_ = yaw_controller_.calc(curr.yaw_, dt, 0);

    ROS_DEBUG("Rotate 3. curr %g, effort %g", curr.yaw_, efforts.yaw_);

    // Continue
    return true;
  }
  else
  {
    // We're done
    return false;
  }
}


void LineMotion::init(const MotionState &curr, const MotionState &goal)
{
  BaseMotion::init(curr, goal);

  double angle_to_goal = atan2(goal.y_ - curr.y_, goal.x_ - curr.x_);

  // Assume instant acceleration
  plan_x_dot_ = XY_VELO * cos(angle_to_goal);
  plan_y_dot_ = XY_VELO * sin(angle_to_goal);

  ROS_DEBUG("Line 1. angle to goal %g, plan_x_dot %g, plan_y_dot %g", angle_to_goal, plan_x_dot_, plan_y_dot_);
}

// Return true to continue, false if we're done
bool LineMotion::advance(const MotionState &curr, OrcaEfforts &efforts)
{
  efforts.clear();

  if (std::hypot(goal_.x_ - curr.x_, goal_.y_ - curr.y_) > XY_EPSILON)
  {
    double dt = (ros::Time::now() - last_time_).toSec();
    last_time_ = ros::Time::now();

    if (std::hypot(goal_.x_ - plan_.x_, goal_.y_ - plan_.y_) > XY_EPSILON)
    {
      // Update our plan
      plan_.x_ += plan_x_dot_ * dt;
      plan_.y_ += plan_y_dot_ * dt;

      x_controller_.setTarget(plan_.x_);
      y_controller_.setTarget(plan_.y_);

      ROS_DEBUG("Line 2. goal_x %g, goal_y %g, plan_x %g, plan_y %g", goal_.x_, goal_.y_, plan_.x_, plan_.y_);
    }

    // Calc desired accelerations
    double x_dot_dot = x_controller_.calc(curr.x_, dt, 0);
    double y_dot_dot = y_controller_.calc(curr.y_, dt, 0);

    // Rotate frame, and assign to efforts
    efforts.forward_ = x_dot_dot * cos(curr.yaw_) + y_dot_dot * sin(curr.yaw_);
    efforts.strafe_ = y_dot_dot * cos(curr.yaw_) - x_dot_dot * sin(curr.yaw_);

    ROS_DEBUG("Line 3. curr_x %g, curr_y %g, yaw %g, x_dot_dot %g, y_dot_dot %g, forward %g, strafe %g", curr.x_, curr.y_,
      curr.yaw_, x_dot_dot, y_dot_dot, efforts.forward_, efforts.strafe_);

    // Hold depth just below the surface
    efforts.vertical_ = z_controller_.calc(curr.depth_, dt, 0);

    // Hold yaw
    // TODO

    // Continue
    return true;
  }
  else
  {
    // We're done
    return false;
  }
}

#if 0
enum class SurfaceMission::Phase
{
  no_goal,    // There's no goal
  plan,       // We're planning (in our case just waiting for a GPS signal)
  turn,       // We're turning toward our goal
  run,        // We're moving toward the goal
  final_turn  // We're turning toward the target heading
};

std::map<SurfaceMission::Phase, std::string> SurfaceMission::phase_names_ =
{
  {SurfaceMission::Phase::no_goal, "no_goal"}, {SurfaceMission::Phase::plan, "plan"},
  {SurfaceMission::Phase::turn, "turn"}, {SurfaceMission::Phase::run, "run"}, {SurfaceMission::Phase::final_turn, "final_turn"}
};

SurfaceMission::SurfaceMission(ros::NodeHandle &nh_priv):
  nh_priv_{nh_priv},
  phase_{Phase::no_goal}
{
  // Subscribe to the GPS sensor
  gps_sub_ = nh_priv_.subscribe<geometry_msgs::Vector3Stamped>("/gps", 10, &SurfaceMission::gpsCallback, this);
}

void SurfaceMission::changePhase(Phase new_phase)
{
  ROS_INFO("mission %s => %s", phase_names_[phase_].c_str(), phase_names_[new_phase].c_str());
  phase_ = new_phase;
}

// Initialize a mission
void SurfaceMission::init(double goal_x, double goal_y, double goal_yaw)
{
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  goal_yaw_ = goal_yaw;

  z_controller_.setTarget(0.5);

  last_time_ = ros::Time::now();

  changePhase(Phase::plan);
}

void SurfaceMission::gpsCallback(const geometry_msgs::Vector3Stamped::ConstPtr& gps_msg)
{
  if (phase_ == Phase::plan)
  {
    curr_x_ = plan_x_ = gps_msg->vector.x;
    curr_y_ = plan_y_ = gps_msg->vector.y;

    plan_x_dot_ = 0;
    plan_y_dot_ = 0;

    changePhase(Phase::turn);
  }
  else
  {
    curr_x_ = gps_msg->vector.x;
    curr_y_ = gps_msg->vector.y;
  }
}

// Calc thruster efforts; return true if we're still running the mission, false if the mission is complete
bool SurfaceMission::advance(double yaw, double depth, double &forward_effort, double &strafe_effort,
  double &vertical_effort, double &yaw_effort)
{
  forward_effort = strafe_effort = vertical_effort = yaw_effort = 0;

  double dt = (ros::Time::now() - last_time_).toSec();
  last_time_ = ros::Time::now();

  switch (phase_)
  {
    case Phase::plan:
    {
      break;
    }

    case Phase::turn:
    {
      double angle_to_goal = atan2(goal_y_ - curr_y_, goal_x_ - curr_x_);

      plan_x_dot_ = VELO * cos(angle_to_goal); // Assume instant acceleration
      plan_y_dot_ = VELO * sin(angle_to_goal); // Assume instant acceleration
      ROS_DEBUG("1. angle to goal %g, plan_x_dot %g, plan_y_dot %g", angle_to_goal, plan_x_dot_, plan_y_dot_);

      changePhase(Phase::run);
      break;
    }

    case Phase::run:
    {
      if (std::hypot(goal_x_ - plan_x_, goal_y_ - plan_y_) > EPSILON)
      {
        // Update our plan
        plan_x_ += plan_x_dot_ * dt;
        plan_y_ += plan_y_dot_ * dt;

        x_controller_.setTarget(plan_x_);
        y_controller_.setTarget(plan_y_);

        ROS_DEBUG("2. goal_x %g, goal_y %g, plan_x %g, plan_y %g",
          goal_x_, goal_y_, plan_x_, plan_y_);
      }

      // Calc desired accelerations
      double x_dot_dot = x_controller_.calc(curr_x_, dt, 0);
      double y_dot_dot = y_controller_.calc(curr_y_, dt, 0);

      // Rotate frame, and assign to efforts
      forward_effort = x_dot_dot * cos(yaw) + y_dot_dot * sin(yaw);
      strafe_effort = y_dot_dot * cos(yaw) - x_dot_dot * sin(yaw);

      ROS_DEBUG("3. curr_x %g, curr_y %g, yaw %g, x_dot_dot %g, y_dot_dot %g, forward %g, strafe %g",
        curr_x_, curr_y_, yaw, x_dot_dot, y_dot_dot, forward_effort, strafe_effort);

      // Hold depth just below the surface
      vertical_effort = z_controller_.calc(depth, dt, 0);

      // Stop if we're w/in epsilon
      if (std::hypot(goal_x_ - curr_x_, goal_y_ - curr_y_) < EPSILON)
      {
        changePhase(Phase::final_turn);
      }
      break;
    }

    case Phase::final_turn:
    {
      plan_x_dot_ = 0; // Assume instant deceleration
      plan_y_dot_ = 0; // Assume instant deceleration
      changePhase(Phase::no_goal);
      return false;
    }

    default:
    {
      ROS_ERROR("No goal, not running");
      return false;
    }
  }
  return true;
}
#endif

} // namespace orca_base