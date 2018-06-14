#ifndef ORCA_MOTION_H
#define ORCA_MOTION_H

#include "orca_base/orca_model.h"
#include "orca_base/pid.h"

namespace orca_base {

constexpr double UNDER_SURFACE = 0.5; // Good depth for running along the surface

//=====================================================================================
// Base motion is a never-ending pid controller
//=====================================================================================

class BaseMotion
{
protected:

  OrcaPose goal_{};

  double x_dot_{0};
  double y_dot_{0};
  double yaw_dot_{0};

  double x_dot_dot_{0};
  double y_dot_dot_{0};
  double yaw_dot_dot_{0};

  pid::Controller x_controller_{false, 0.8, 0.25};
  pid::Controller y_controller_{false, 0.8, 0.25};
  pid::Controller z_controller_{false, 0.8, 0.25};
  pid::Controller yaw_controller_{true, 0.09, 0, 0.03}; // TODO

public:

  // Initialize the motion plan, return true if successful
  virtual bool init(const OrcaPose &start, const OrcaPose &goal);

  // Advance the motion plan, return true to continue, false if we're done
  virtual bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts);
};

//=====================================================================================
// Rotate about a point
//=====================================================================================

class RotateMotion: public BaseMotion
{
public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts) override;
};

//=====================================================================================
// Move from point A to point B
//=====================================================================================

class LineMotion: public BaseMotion
{
public:

  bool init(const OrcaPose &start, const OrcaPose &goal) override;
  bool advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts) override;
};

} // namespace orca_base

#endif // ORCA_MOTION_H