#include "orca_base/orca_motion.h"

namespace orca_base {

//=====================================================================================
// Constants
//=====================================================================================

constexpr double VELO_XY = 0.5;                 // Velocity for xy motion (m/s)
constexpr double VELO_Z = 0.3;                  // Velocity for z motion (m/s)
constexpr double EPSILON_PLAN_XYZ = 0.05;       // Close enough for xyz motion (m)
constexpr double VELO_YAW = M_PI / 10;          // Rotation velocity (r/s)
constexpr double EPSILON_PLAN_YAW = M_PI / 90;  // Close enough for yaw motion (r)

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

bool BaseMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  // Set goal
  goal_ = goal;

  // Init state
  pose_ = start;
  velo_ = OrcaPose{};
  ff_ = OrcaPose{};

  // Init PID controllers
  x_controller_.setTarget(pose_.x);
  y_controller_.setTarget(pose_.y);
  z_controller_.setTarget(pose_.z);
  yaw_controller_.setTarget(pose_.yaw);

  // Always succeed
  return true;
}

bool BaseMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  // Return planned pose
  plan = pose_;

  // u_bar is required acceleration
  OrcaPose u_bar{};
  u_bar.x = x_controller_.calc(curr.x, dt, ff_.x);
  u_bar.y = y_controller_.calc(curr.y, dt, ff_.y);
  u_bar.z = z_controller_.calc(curr.z, dt, ff_.z);
  u_bar.yaw = yaw_controller_.calc(curr.yaw, dt, ff_.yaw);

  // u_bar (acceleration) => u (control inputs normalized from -1 to 1, aka effort)
  double x_effort = accel_to_effort_xy(u_bar.x);
  double y_effort = accel_to_effort_xy(u_bar.y);
  double z_effort = accel_to_effort_z(u_bar.z);
  efforts.yaw = accel_to_effort_yaw(u_bar.yaw);

  // Convert from world frame to body frame
  efforts.vertical = -z_effort; // TODO ???
  rotate_frame(x_effort, y_effort, curr.yaw, efforts.forward, efforts.strafe);

  // Never stop
  return true;
}

//=====================================================================================
// RotateMotion
//=====================================================================================

bool RotateMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  BaseMotion::init(start, goal);

  if (start.distance_xy(goal) > EPSILON_PLAN_XYZ || start.distance_z(goal) > EPSILON_PLAN_XYZ)
  {
    ROS_ERROR("Can't init rotate motion: x, y and z must be the same");
    return false;
  }

  // Pick the shortest direction
  velo_.yaw = norm_angle(goal.yaw - start.yaw) > 0 ? VELO_YAW : -VELO_YAW;

  // Drag torque => thrust torque => acceleration => feedforward
  ff_.yaw = torque_to_accel_yaw(-drag_torque_yaw(velo_.yaw));

  ROS_DEBUG("Rotate init: start %g, goal %g, velocity %g, accel %g", start.yaw, goal.yaw, velo_.yaw, ff_.yaw);
  return true;
}

bool RotateMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  if (goal_.distance_yaw(pose_) > EPSILON_PLAN_YAW)
  {
    // Update pose
    pose_.yaw = norm_angle(pose_.yaw + velo_.yaw * dt);

    // Set targets
    yaw_controller_.setTarget(pose_.yaw);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan = goal_;
    return false;
  }
}

//=====================================================================================
// LineMotion
//=====================================================================================

bool LineMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  BaseMotion::init(start, goal);

  if (start.distance_z(goal) > EPSILON_PLAN_XYZ || start.distance_yaw(goal) > EPSILON_PLAN_YAW)
  {
    ROS_ERROR("Can't init line motion: z and yaw must be the same");
    return false;
  }

  // Compute angle start => goal
  double angle_to_goal = atan2(goal.y - start.y, goal.x - start.x);

  // Drag force => thrust force => acceleration => feedforward
  drag_force_to_accel_xy(goal.yaw, VELO_XY * cos(angle_to_goal), VELO_XY * sin(angle_to_goal), ff_.x, ff_.y);

  ROS_DEBUG("Line init: start (%g, %g), goal (%g, %g), ff (%g, %g)", start.x, start.y, goal.x, goal.y, ff_.x, ff_.y);
  return true;
}

bool LineMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  double distance_remaining = goal_.distance_xy(pose_);
  if (distance_remaining > EPSILON_PLAN_XYZ)
  {
    if (distance_remaining - deceleration_distance(goal_.yaw, velo_.x, velo_.y) < EPSILON_PLAN_XYZ)
    {
      // Decelerate
      ff_.x = ff_.y = 0;
    }

    // Compute acceleration due to drag
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(goal_.yaw, velo_.x, velo_.y, accel_drag_x, accel_drag_y);

    // Update velocity
    velo_.x += (ff_.x - accel_drag_x) * dt;
    velo_.y += (ff_.y - accel_drag_y) * dt;

    // Update pose
    pose_.x += velo_.x * dt;
    pose_.y += velo_.y * dt;

    // Set targets
    x_controller_.setTarget(pose_.x);
    y_controller_.setTarget(pose_.y);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // Close enough, we're done
    plan = goal_;
    efforts.clear();
    return false;
  }
}

//=====================================================================================
// ArcMotion
//=====================================================================================

bool ArcMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  BaseMotion::init(start, goal);

  if (start.distance_z(goal) > EPSILON_PLAN_XYZ)
  {
    ROS_ERROR("Can't init arc motion: z must be the same");
    return false;
  }

  // Calc radius
  arc_.radius_ = std::hypot(goal.x - start.x, goal.y - start.y) / 2;
  if (arc_.radius_ < 1)
  {
    ROS_ERROR("Can't init arc motion: radius is too small");
    return false;
  }

  // Calc center
  arc_.center_.x = start.x - sin(start.yaw) * arc_.radius_;
  arc_.center_.y = start.y + cos(start.yaw) * arc_.radius_;

  // Init polar start, goal angles
  arc_.start_angle_ = norm_angle(start.yaw - M_PI_2);
  arc_.goal_angle_ = norm_angle(goal.yaw - M_PI_2);

  // Init polar pose and velocity
  polar_pose_ = arc_.start_angle_;
  polar_velo_ = VELO_XY / arc_.radius_;

  // Drag torque => thrust torque => acceleration => feedforward
  velo_.yaw = polar_velo_;
  ff_.yaw = torque_to_accel_yaw(-drag_torque_yaw(velo_.yaw));

  ROS_DEBUG("Arc init: center (%g, %g), radius %g, velo yaw %g, ff yaw %g", arc_.center_.x, arc_.center_.y, arc_.radius_, velo_.yaw, ff_.yaw);
  return true;
}

bool ArcMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  if (std::abs(norm_angle(arc_.goal_angle_ - polar_pose_)) > EPSILON_PLAN_YAW)
  {
    OrcaPose previous_pose = pose_;
    OrcaPose previous_velo = velo_;

    // Update pose in polar coordinates
    polar_pose_ += polar_velo_ * dt;

    // Convert polar to cartesian
    arc_.polarToCartesian(polar_pose_, pose_);

    // Planned velocity
    velo_.x = (pose_.x - previous_pose.x) / dt;
    velo_.y = (pose_.y - previous_pose.y) / dt;

    // Planned acceleration
    double accel_x = (velo_.x - previous_velo.x) / dt;
    double accel_y = (velo_.y - previous_velo.y) / dt;

    // Drag force => thrust force => acceleration
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(pose_.yaw, velo_.x, velo_.y, accel_drag_x, accel_drag_y);

    // Add both accelerations
    ff_.x = accel_x + accel_drag_x;
    ff_.y = accel_y + accel_drag_y;

    // Set targets
    x_controller_.setTarget(pose_.x);
    y_controller_.setTarget(pose_.y);
    yaw_controller_.setTarget(pose_.yaw);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // Close enough, we're done
    plan = goal_;
    efforts.clear();
    return false;
  }
}

//=====================================================================================
// VerticalMotion
//=====================================================================================

bool VerticalMotion::init(const OrcaPose &start, const OrcaPose &goal)
{
  BaseMotion::init(start, goal);

  if (start.distance_xy(goal) > EPSILON_PLAN_XYZ || start.distance_yaw(goal) > EPSILON_PLAN_YAW || goal.z > 0)
  {
    ROS_ERROR("Can't init vertical motion: x, y and yaw must be the same, goal.z must be negative");
    return false;
  }

  // Ascend (+) or descend (-)
  double direction = goal.z > start.z ? 1 : -1;

  // Target velocity
  velo_.z = direction * VELO_Z;

  // Drag force => thrust force => acceleration => feedforward
  ff_.z = direction * force_to_accel_z(-drag_force_z(VELO_Z));

  ROS_DEBUG("Vertical init: start %g, goal %g, velocity %g, ff %g", start.z, goal.z, velo_.z, ff_.z);
  return true;
}

bool VerticalMotion::advance(double dt, const orca_base::OrcaPose &curr, orca_base::OrcaPose &plan, orca_base::OrcaEfforts &efforts)
{
  if (goal_.distance_z(pose_) > EPSILON_PLAN_XYZ)
  {
    // Update pose
    pose_.z += velo_.z * dt;

    // Set targets
    z_controller_.setTarget(pose_.z);

    ROS_DEBUG("Vertical advance: plan %g, curr %g", pose_.z, curr.z);

    // Compute efforts
    BaseMotion::advance(dt, curr, plan, efforts);
    return true;
  }
  else
  {
    // We're done
    plan = goal_;
    return false;
  }
}

} // namespace orca_base