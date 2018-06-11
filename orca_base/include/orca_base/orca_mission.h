#ifndef ORCA_MISSION_H
#define ORCA_MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "orca_base/orca_model.h"
#include "orca_base/pid.h"

namespace orca_base {

constexpr const double UNDER_SURFACE = 0.5; // Good depth for running along the surface

// Abstract base class for motion prediction
class BaseMotion
{
protected:

  MotionState goal_;      // Goal state

public:

  // Initialize the motion prediction
  virtual void init(const MotionState &start, const MotionState &goal, MotionState &plan);

  // Advance the motion prediction; return true to continue, false if we're done
  virtual bool advance(double dt, MotionState &plan) = 0;
};

// Rotate about a point.
class RotateMotion : public BaseMotion
{
public:

  void init(const MotionState &start, const MotionState &goal, MotionState &plan) override;
  bool advance(double dt, MotionState &plan) override;
};

// Move in a straight line from one pose to another.
class LineMotion : public BaseMotion
{
public:

  void init(const MotionState &start, const MotionState &goal, MotionState &plan) override;
  bool advance(double dt, MotionState &plan) override;
};

// TODO circle -- need radius in goal
// TODO box -- need x, y box size in goal
// TODO submerge and surface

class SurfaceMission
{
private:

  enum class Phase
  {
    no_goal,    // There's no goal
    turn,       // We're turning toward our goal
    run,        // We're moving toward the goal
    final_turn  // We're turning toward the target heading
  };

  Phase phase_ = Phase::no_goal;

  MotionState goal1_;
  MotionState goal2_;
  MotionState goal3_;

  std::unique_ptr<BaseMotion> predictor_;

  MotionState plan_;

  ros::Time last_time_;   // Time of last call to advance
  double dt_;             // Elapsed time since the last call to advance (s)

  pid::Controller x_controller_{false, 0.03, 0, 0.01};
  pid::Controller y_controller_{false, 0.03, 0, 0.01};
  pid::Controller z_controller_{false, 0.05, 0, 0.05};
  pid::Controller yaw_controller_{true, 0.09, 0, 0.03};

public:

  void init(const MotionState &start, const MotionState &goal);
  bool advance(const MotionState &curr, OrcaEfforts &efforts);
};

} // namespace orca_base

#endif // ORCA_MISSION_H