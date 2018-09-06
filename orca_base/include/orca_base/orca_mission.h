#ifndef ORCA_MISSION_H
#define ORCA_MISSION_H

#include <nav_msgs/Path.h>
#include "orca_base/orca_motion.h"

namespace orca_base {

//=====================================================================================
// BaseMission is an abstract base class for running missions
//=====================================================================================

class BaseMission
{
protected:

  std::unique_ptr<BaseMotion> planner_;
  //OrcaPose pose_;

  ros::Time last_time_;   // Time of last call to advance
  double dt_;             // Elapsed time since the last call to advance (s)

public:

  virtual bool init(const OrcaPose &goal, OrcaOdometry &plan) = 0;
  virtual bool advance(const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts);

  static void addToPath(nav_msgs::Path &path, const OrcaPose &pose);
  static void addToPath(nav_msgs::Path &path, const std::vector<OrcaPose> &poses);
};

//=====================================================================================
// SurfaceMission
//
// Run from pose A to B
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

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts) override;
};

//=====================================================================================
// SquareMission
//
// Run in a square defined by 2 poses
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

class SquareMission: public BaseMission
{
private:

  enum class Planner
  {
    vertical,
    rotate,
    line
  };

  struct Segment
  {
    Planner planner;
    OrcaPose goal;
  };

  int segment_;
  std::vector<Segment>segments_;

public:

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts) override;
};

//=====================================================================================
// ArcMission
//
// Run in an arc defined by 2 poses
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

class ArcMission: public BaseMission
{
public:

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts) override;
};

//=====================================================================================
// VerticalMission
//
// Descend or ascend
//=====================================================================================

class VerticalMission: public BaseMission
{
public:

  bool init(const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts) override;
};

} // namespace orca_base

#endif // ORCA_MISSION_H