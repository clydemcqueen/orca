#include "orca_base/orca_motion.h"

namespace orca_base {

//=====================================================================================
// Utilities
//=====================================================================================

// Compute acceleration required to counteract drag force
void drag_force_to_accel_xy(const double yaw, const double x_v, const double y_v, double &x_a, double &y_a)
{
  // Rotate velocity into the body frame
  double forward_v, strafe_v;
  rotate_frame(x_v, y_v, yaw, forward_v, strafe_v);

  // Calc acceleration due to drag force
  double forward_a = force_to_accel_xy(-drag_force_x(forward_v));
  double strafe_a = force_to_accel_xy(-drag_force_y(strafe_v));

  // Rotate back
  rotate_frame(forward_a, strafe_a, -yaw, x_a, y_a);
}

// Compute the deceleration (glide) distance
double deceleration_distance(const double yaw, double velo_x, double velo_y)
{
  constexpr double dt = 0.1;
  double x = 0;
  double y = 0;

  for (double t = 0; t < 10; t += dt)
  {
    // Compute drag force
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(yaw, velo_x, velo_y, accel_drag_x, accel_drag_y);

    // Update velocity
    velo_x -= accel_drag_x * dt;
    velo_y -= accel_drag_y * dt;

    // Update distance
    x += velo_x * dt;
    y += velo_y * dt;

    if (std::hypot(velo_x, velo_y) < 0.1)
    {
      // Close enough
      return std::hypot(x, y);
    }
  }

  ROS_ERROR("Gliding > 10 seconds? Unlikely");
  return 0;
}

//=====================================================================================
// BaseMotion
//=====================================================================================

bool BaseMotion::init(const OrcaPose &goal, OrcaOdometry &plan)
{
  // Set goal
  goal_ = goal;

  // Init state
  plan.stopMotion();
  ff_ = OrcaPose{};

  // Init PID controllers
  x_controller_.setTarget(plan.pose.x);
  y_controller_.setTarget(plan.pose.y);
  z_controller_.setTarget(plan.pose.z);
  yaw_controller_.setTarget(plan.pose.yaw);

  // Always succeed
  return true;
}

bool BaseMotion::advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts)
{
  // u_bar is required acceleration
  OrcaPose u_bar{};
#if 0
  u_bar.x = x_controller_.calc(curr.x, dt, ff_.x);
  u_bar.y = y_controller_.calc(curr.y, dt, ff_.y);
  u_bar.z = z_controller_.calc(curr.z, dt, ff_.z);
  u_bar.yaw = yaw_controller_.calc(curr.yaw, dt, ff_.yaw);
#else
  // TODO temp while debugging -- remove this
  u_bar.x = ff_.x;
  u_bar.y = ff_.y;
  u_bar.z = ff_.z;
  u_bar.yaw = ff_.yaw;
#endif

  // u_bar (acceleration) => u (control inputs normalized from -1 to 1, aka effort)
  double x_effort = accel_to_effort_xy(u_bar.x);
  double y_effort = accel_to_effort_xy(u_bar.y);
  double z_effort = accel_to_effort_z(u_bar.z);
  efforts.yaw = accel_to_effort_yaw(u_bar.yaw);

  // Convert from world frame to body frame
  efforts.vertical = -z_effort;
  rotate_frame(x_effort, y_effort, curr.yaw, efforts.forward, efforts.strafe);

  // Update pose covariance
  for (int i = 0; i < 4; ++i)
  {
    plan.pose_covariance[i + 4 * i] += DEF_COVAR * dt;
  }

  // Never stop
  return true;
}

//=====================================================================================
// RotateMotion
//=====================================================================================

bool RotateMotion::init(const OrcaPose &goal, OrcaOdometry &plan)
{
  BaseMotion::init(goal, plan);

  if (plan.pose.distance_xy(goal) > EPSILON_PLAN_XYZ || plan.pose.distance_z(goal) > EPSILON_PLAN_XYZ)
  {
    ROS_ERROR("Can't init rotate motion: x, y and z must be the same");
    return false;
  }

  // Pick the shortest direction
  plan.velo.yaw = norm_angle(goal.yaw - plan.pose.yaw) > 0 ? VELO_YAW : -VELO_YAW;

  // Drag torque => thrust torque => acceleration => feedforward
  ff_.yaw = torque_to_accel_yaw(-drag_torque_yaw(plan.velo.yaw));
  ff_.z = HOVER_ACCEL_Z;

  ROS_DEBUG("Rotate init: start %g, goal %g, velocity %g, accel %g", plan.pose.yaw, goal.yaw, plan.velo.yaw, ff_.yaw);
  return true;
}

bool RotateMotion::advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts)
{
  if (goal_.distance_yaw(plan.pose) > EPSILON_PLAN_YAW)
  {
    // Update pose
    plan.pose.yaw = norm_angle(plan.pose.yaw + plan.velo.yaw * dt);

    // Set targets
    yaw_controller_.setTarget(plan.pose.yaw);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan.pose = goal_;
    plan.stopMotion();
    efforts.clear();
    return false;
  }
}

//=====================================================================================
// LineMotion
//=====================================================================================

bool LineMotion::init(const OrcaPose &goal, OrcaOdometry &plan)
{
  BaseMotion::init(goal, plan);

  if (plan.pose.distance_z(goal) > EPSILON_PLAN_XYZ || plan.pose.distance_yaw(goal) > EPSILON_PLAN_YAW)
  {
    ROS_ERROR("Can't init line motion: z and yaw must be the same");
    return false;
  }

  // Compute angle start => goal
  double angle_to_goal = atan2(goal.y - plan.pose.y, goal.x - plan.pose.x);

  // Drag force => thrust force => acceleration => feedforward
  drag_force_to_accel_xy(goal.yaw, VELO_XY * cos(angle_to_goal), VELO_XY * sin(angle_to_goal), ff_.x, ff_.y);
  ff_.z = HOVER_ACCEL_Z;

  ROS_DEBUG("Line init: start (%g, %g, %g), goal (%g, %g, %g), ff (%g, %g, %g)", plan.pose.x, plan.pose.y, plan.pose.z, goal.x, goal.y, goal.z, ff_.x, ff_.y, ff_.z);
  return true;
}

bool LineMotion::advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts)
{
  double distance_remaining = goal_.distance_xy(plan.pose);
  if (distance_remaining > EPSILON_PLAN_XYZ)
  {
    if (distance_remaining - deceleration_distance(goal_.yaw, plan.velo.x, plan.velo.y) < EPSILON_PLAN_XYZ)
    {
      // Decelerate
      ff_.x = ff_.y = 0;
    }

    // Compute acceleration due to drag
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(goal_.yaw, plan.velo.x, plan.velo.y, accel_drag_x, accel_drag_y);

    // Update velocity
    plan.velo.x += (ff_.x - accel_drag_x) * dt;
    plan.velo.y += (ff_.y - accel_drag_y) * dt;

    // Update pose
    plan.pose.x += plan.velo.x * dt;
    plan.pose.y += plan.velo.y * dt;

    // Set targets
    x_controller_.setTarget(plan.pose.x);
    y_controller_.setTarget(plan.pose.y);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan.pose = goal_;
    plan.stopMotion();
    efforts.clear();
    return false;
  }
}

//=====================================================================================
// ArcMotion
//=====================================================================================

bool ArcMotion::init(const OrcaPose &goal, OrcaOdometry &plan)
{
  BaseMotion::init(goal, plan);

  if (plan.pose.distance_z(goal) > EPSILON_PLAN_XYZ)
  {
    ROS_ERROR("Can't init arc motion: z must be the same");
    return false;
  }

  // Calc radius
  arc_.radius_ = std::hypot(goal.x - plan.pose.x, goal.y - plan.pose.y) / 2;
  if (arc_.radius_ < 1)
  {
    ROS_ERROR("Can't init arc motion: radius is too small");
    return false;
  }

  // Calc center
  arc_.center_.x = plan.pose.x - sin(plan.pose.yaw) * arc_.radius_;
  arc_.center_.y = plan.pose.y + cos(plan.pose.yaw) * arc_.radius_;

  // Init polar start, goal angles
  arc_.start_angle_ = norm_angle(plan.pose.yaw - M_PI_2);
  arc_.goal_angle_ = norm_angle(goal.yaw - M_PI_2);

  // Init polar pose and velocity
  polar_pose_ = arc_.start_angle_;
  polar_velo_ = VELO_XY / arc_.radius_;

  // Drag torque => thrust torque => acceleration => feedforward
  plan.velo.yaw = polar_velo_;
  ff_.yaw = torque_to_accel_yaw(-drag_torque_yaw(plan.velo.yaw));
  ff_.z = HOVER_ACCEL_Z;

  ROS_DEBUG("Arc init: center (%g, %g), radius %g, velo yaw %g, ff yaw %g", arc_.center_.x, arc_.center_.y, arc_.radius_, plan.velo.yaw, ff_.yaw);
  return true;
}

bool ArcMotion::advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts)
{
  if (std::abs(norm_angle(arc_.goal_angle_ - polar_pose_)) > EPSILON_PLAN_YAW)
  {
    OrcaPose previous_pose = plan.pose;
    OrcaPose previous_velo = plan.velo;

    // Update pose in polar coordinates
    polar_pose_ += polar_velo_ * dt;

    // Convert polar to cartesian
    arc_.polarToCartesian(polar_pose_, plan.pose);

    // Planned velocity
    plan.velo.x = (plan.pose.x - previous_pose.x) / dt;
    plan.velo.y = (plan.pose.y - previous_pose.y) / dt;

    // Planned acceleration
    double accel_x = (plan.velo.x - previous_velo.x) / dt;
    double accel_y = (plan.velo.y - previous_velo.y) / dt;

    // Drag force => thrust force => acceleration
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(plan.pose.yaw, plan.velo.x, plan.velo.y, accel_drag_x, accel_drag_y);

    // Add both accelerations
    ff_.x = accel_x + accel_drag_x;
    ff_.y = accel_y + accel_drag_y;

    // Set targets
    x_controller_.setTarget(plan.pose.x);
    y_controller_.setTarget(plan.pose.y);
    yaw_controller_.setTarget(plan.pose.yaw);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan.pose = goal_;
    plan.stopMotion();
    efforts.clear();
    return false;
  }
}

//=====================================================================================
// VerticalMotion
//=====================================================================================

bool VerticalMotion::init(const OrcaPose &goal, OrcaOdometry &plan)
{
  BaseMotion::init(goal, plan);

  if (plan.pose.distance_xy(goal) > EPSILON_PLAN_XYZ || plan.pose.distance_yaw(goal) > EPSILON_PLAN_YAW || goal.z > 0)
  {
    ROS_ERROR("Can't init vertical motion: x, y and yaw must be the same, goal.z must be negative");
    return false;
  }

  // Ascend (+) or descend (-)
  double direction = goal.z > plan.pose.z ? 1 : -1;

  // Target velocity
  plan.velo.z = direction * VELO_Z;

  // Drag force => thrust force => acceleration => feedforward
  ff_.z = direction * force_to_accel_z(-drag_force_z(VELO_Z)) + HOVER_ACCEL_Z;

  ROS_DEBUG("Vertical init: start %g, goal %g, velocity %g, ff %g", plan.pose.z, goal.z, plan.velo.z, ff_.z);
  return true;
}

bool VerticalMotion::advance(double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaEfforts &efforts)
{
  if (goal_.distance_z(plan.pose) > EPSILON_PLAN_XYZ)
  {
    // Update pose
    plan.pose.z += plan.velo.z * dt;

    // Set targets
    z_controller_.setTarget(plan.pose.z);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan.pose = goal_;
    plan.stopMotion();
    efforts.clear();
    return false;
  }
}

} // namespace orca_base