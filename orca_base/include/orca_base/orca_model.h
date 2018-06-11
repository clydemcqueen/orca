#ifndef ORCA_MODEL_H
#define ORCA_MODEL_H

namespace orca_base {

// Motion state variables
struct MotionState // TODO add velocities
{
  // Everything is ENU. Yaw == 0 when facing east.

  double x_;
  double x_dot_;

  double y_;
  double y_dot_;

  double depth_;

  double yaw_;
  double yaw_dot_;

  // TODO add altitude? Odometry might need constant altitude, but variable depth

  MotionState(): x_(0), x_dot_(0), y_(0), y_dot_(0), depth_(0), yaw_(0), yaw_dot_(0) {}

  MotionState(double x, double y, double depth, double yaw): MotionState()
  {
    x_ = x;
    y_ = y;
    depth_ = depth;
    yaw_ = yaw;
  }
};

// Thruster efforts from joystick or pid controllers (yaw and depth), ranges from 1.0 for forward to -1.0 for reverse
struct OrcaEfforts
{
  double forward_;
  double strafe_;
  double vertical_;
  double yaw_;

  OrcaEfforts(): forward_(0), strafe_(0), vertical_(0), yaw_(0) {}
  OrcaEfforts(double forward, double strafe, double depth, double yaw): forward_(forward), strafe_(strafe), vertical_(depth), yaw_(yaw) {}

  void clear() { forward_ = 0; strafe_ = 0; vertical_ = 0; yaw_ = 0; }
};

} // namespace orca_base

#endif // ORCA_MODEL_H