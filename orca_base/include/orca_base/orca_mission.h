#ifndef ORCA_MISSION_H
#define ORCA_MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "orca_base/orca_model.h"
#include "orca_base/pid.h"

namespace orca_base {

constexpr const double SURFACE_DEPTH = 0.5; // Good depth for running along the surface

// Abstract base class for motion prediction
class BaseMotion
{
protected:

  MotionState goal_;      // Goal state
  MotionState plan_;      // Our evolving plan to get to the goal

  ros::Time last_time_;   // Time of last call to advance
  double dt_;             // Elapsed time since the last call to advance (s)

  pid::Controller x_controller_{false, 0.03, 0, 0.01};
  pid::Controller y_controller_{false, 0.03, 0, 0.01};
  pid::Controller z_controller_{false, 0.05, 0, 0.05};
  pid::Controller yaw_controller_{true, 0.09, 0, 0.03};

public:

  // Initialize the motion prediction
  virtual void init(const MotionState &curr, const MotionState &goal);

  // Advance the motion prediction; return true to continue, false if we're done
  virtual bool advance(const MotionState &curr, OrcaEfforts &efforts);
};

// Rotate about a point.
class RotateMotion : BaseMotion
{
private:

  double plan_yaw_dot_;

public:

  void init(const MotionState &curr, const MotionState &goal) override;
  bool advance(const MotionState &curr, OrcaEfforts &efforts) override;
};

// Move in a straight line from one pose to another.
class LineMotion : BaseMotion
{
private:

  double plan_x_dot_;
  double plan_y_dot_;

public:

  void init(const MotionState &curr, const MotionState &goal) override;
  bool advance(const MotionState &curr, OrcaEfforts &efforts) override;
};

// TODO composite, with rotate + line + rotate -- different goal state
// TODO circle -- need radius in goal state
// TODO composite grid -- need x, y grid size in goal state
// TODO submerge and surface

#if 0
class SurfaceMission
{
private:

  enum class Phase:int;
  static std::map<Phase, std::string> phase_names_;

  ros::NodeHandle &nh_priv_;
  Phase phase_;
  ros::Subscriber gps_sub_;
  ros::Time last_time_;
  pid::Controller x_controller_{false, 0.03, 0, 0.01};
  pid::Controller y_controller_{false, 0.03, 0, 0.01};
  pid::Controller z_controller_{false, 0.05, 0, 0.05};

  // Goal state
  double goal_x_ = 0;
  double goal_y_ = 0;
  double goal_yaw_ = 0;

  // Planned state
  double plan_x_ = 0;
  double plan_y_ = 0;
  double plan_x_dot_ = 0;
  double plan_y_dot_ = 0;

  // Current state
  double curr_x_ = 0;
  double curr_y_ = 0;

  void changePhase(Phase new_phase);

  void gpsCallback(const geometry_msgs::Vector3Stamped::ConstPtr& gps_msg);

public:

  SurfaceMission(ros::NodeHandle &nh_priv);

  // Initialize a mission
  void init(double goal_x, double goal_y, double goal_yaw);

  // Calc thruster efforts; return true if we're still running the mission, false if the mission is complete
  bool advance(double yaw, double depth, double &forward_effort, double &strafe_effort, double &vertical_effort, double &yaw_effort);
};
#endif

} // namespace orca_base

#endif // ORCA_MISSION_H