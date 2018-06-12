#ifndef ORCA_MODEL_H
#define ORCA_MODEL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace orca_base {

//=====================================================================================
// 4 DoF pose
// Everything is ENU, yaw == 0 when facing east
//=====================================================================================

struct OrcaPose
{
  double x_;
  double y_;
  double depth_;
  double yaw_;

  OrcaPose(): x_(0), y_(0), depth_(0), yaw_(0) {}
  OrcaPose(double x, double y, double depth, double yaw): x_(x), y_(y), depth_(depth), yaw_(yaw) {}

  void toMsg(geometry_msgs::Pose &msg) const
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    msg.orientation = tf2::toMsg(q);

    msg.position.x = x_;
    msg.position.y = y_;
    msg.position.z = -depth_;
  }
};

//=====================================================================================
// Thruster efforts from joystick or pid controllers
// Ranges from 1.0 for forward to -1.0 for reverse
//=====================================================================================

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

//=====================================================================================
// Utils
//=====================================================================================

// Move an angle to the region [-M_PI, M_PI]
constexpr double norm_angle(double a)
{
  while (a < -M_PI)
  {
    a += 2 * M_PI;
  }
  while (a > M_PI)
  {
    a -= 2 * M_PI;
  }

  return a;
}

// Distance between 2 poses (m)
constexpr double distance_xy(const OrcaPose &a, const OrcaPose &b)
{
  return std::hypot(a.x_ - b.x_, a.y_ - b.y_);
}

// Distance between 2 yaw angles (r)
constexpr double distance_yaw(const OrcaPose &a, const OrcaPose &b)
{
  return std::abs(norm_angle(a.yaw_ - b.yaw_));
}

} // namespace orca_base

#endif // ORCA_MODEL_H