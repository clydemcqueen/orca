#ifndef ORCA_MISSION_H
#define ORCA_MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include "orca_base/orca_model.h"
#include "orca_base/pid.h"

namespace orca_base {

constexpr const double UNDER_SURFACE = 0.5; // Good depth for running along the surface

//=====================================================================================
// Abstract base class for motion planning
//=====================================================================================

class BaseMotion
{
protected:

  OrcaPose goal_;      // Goal pose

public:

  // Initialize the motion plan, return true if successful
  virtual bool init(const OrcaPose &start, const OrcaPose &goal);

  // Advance the motion plan, return true to continue, false if we're done
  virtual bool advance(double dt, OrcaPose &plan) = 0;
};

//=====================================================================================
// Plan motion about a point
//=====================================================================================

class RotateMotion: public BaseMotion
{
private:

  double yaw_dot_;

public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, OrcaPose &plan) override;
};

//=====================================================================================
// Plan motion from point A to point B
//=====================================================================================

class LineMotion: public BaseMotion
{
private:

  double x_dot_;
  double y_dot_;

public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, OrcaPose &plan) override;
};

//=====================================================================================
// Abstract base class for running missions
//=====================================================================================

class BaseMission
{
protected:

  std::unique_ptr<BaseMotion> planner_;
  OrcaPose plan_;

  ros::Time last_time_;   // Time of last call to advance
  double dt_;             // Elapsed time since the last call to advance (s)

  pid::Controller x_controller_{false, 0.03, 0, 0.01};
  pid::Controller y_controller_{false, 0.03, 0, 0.01};
  pid::Controller z_controller_{false, 0.05, 0, 0.05};
  pid::Controller yaw_controller_{true, 0.09, 0, 0.03};

public:

  virtual bool init(const OrcaPose &start, const OrcaPose &goal, nav_msgs::Path &path);
  virtual bool advance(const OrcaPose &curr, OrcaEfforts &efforts);

  static void addToPath(nav_msgs::Path &path, const OrcaPose &pose);
  static void addToPath(nav_msgs::Path &path, const std::vector<OrcaPose> &poses);
};

//=====================================================================================
// SurfaceMission
//
// Run from point A to B at the surface
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

class SurfaceMission: public BaseMission
{
private:

  enum class Phase
  {
    no_goal,    // There's no goal
    turn,       // We're turning toward the goal
    run,        // We're moving toward the goal
    final_turn  // We're turning to match the goal heading
  };

  Phase phase_ = Phase::no_goal;

  OrcaPose goal1_;
  OrcaPose goal2_;
  OrcaPose goal3_;

public:

  bool init(const OrcaPose &start, const OrcaPose &goal, nav_msgs::Path &path) override;
  bool advance(const OrcaPose &curr, OrcaEfforts &efforts) override;
};

//=====================================================================================
// SquareMission
//
// Run in a square defined by 2 points
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

class SquareMission: public BaseMission
{
private:

  constexpr const static int NO_GOAL = -1;
  int phase_ = NO_GOAL;
  std::vector<OrcaPose>goals_;

public:

  bool init(const OrcaPose &start, const OrcaPose &goal, nav_msgs::Path &path) override;
  bool advance(const OrcaPose &curr, OrcaEfforts &efforts) override;
};

} // namespace orca_base

#endif // ORCA_MISSION_H