#include "sub_robot_core/sub_controller.hpp"

#include <cmath>

namespace sub_robot_core
{

SubController::SubController(rclcpp::Node * node)
: node_(node)
{
  auto declare = [this](const std::string & name, double default_value) {
    return node_->declare_parameter<double>(name, default_value);
  };

  params_.target_distance = declare("target_distance", params_.target_distance);
  params_.align_offset_y = declare("align_offset_y", params_.align_offset_y);
  params_.align_offset_x = declare("align_offset_x", params_.align_offset_x);
  params_.tol_xy = declare("tol_xy", params_.tol_xy);
  params_.tol_yaw = declare("tol_yaw", params_.tol_yaw);
  params_.max_lin = declare("max_lin", params_.max_lin);
  params_.max_ang = declare("max_ang", params_.max_ang);
  params_.odom_timeout = declare("odom_timeout", params_.odom_timeout);
  params_.k_lin = declare("k_lin", params_.k_lin);
  params_.k_lat = declare("k_lat", params_.k_lat);
  params_.k_yaw = declare("k_yaw", params_.k_yaw);

  RCLCPP_INFO(node_->get_logger(),
    "SubController parameters - target_distance: %.2f, align_offset(x,y): (%.2f, %.2f), "
    "tolerance(xy,yaw): (%.2f, %.2f), max_vel(lin,ang): (%.2f, %.2f), odom_timeout: %.2f",
    params_.target_distance,
    params_.align_offset_x, params_.align_offset_y,
    params_.tol_xy, params_.tol_yaw,
    params_.max_lin, params_.max_ang,
    params_.odom_timeout);

  apply_stop();
}

void SubController::handle_odom(const nav_msgs::msg::Odometry & msg)
{
  last_odom_ = msg;
  last_odom_time_ = msg.header.stamp;
  has_odom_ = true;
  if (!estimate_initialised_) {
    initialise_estimate();
  }
  RCLCPP_DEBUG(node_->get_logger(), "Received /main/odom at %.3f", last_odom_time_.seconds());
}

void SubController::handle_command(uint8_t command)
{
  RCLCPP_INFO(node_->get_logger(), "Received task command: %u", command);
  switch (command) {
    case 0:
      to_state(FsmState::STOP);
      break;
    case 1:
      to_state(FsmState::FOLLOW);
      break;
    case 2:
      to_state(FsmState::TASK_WAIT);
      break;
    case 3:
      to_state(FsmState::FORMATION_ALIGN);
      break;
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown command %u", command);
      break;
  }
}

void SubController::update(const rclcpp::Time & now)
{
  double dt = 0.0;
  if (estimate_initialised_) {
    dt = (now - last_update_time_).seconds();
    if (dt < 0.0) {
      dt = 0.0;
    }
    integrate_estimate(dt);
  }
  last_update_time_ = now;

  if (!has_odom_ || odom_stale(now)) {
    if (state_ != FsmState::STOP) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Odom timeout, stopping");
      to_state(FsmState::STOP);
    }
    apply_stop();
    return;
  }

  switch (state_) {
    case FsmState::STOP:
      apply_stop();
      break;
    case FsmState::FOLLOW:
      compute_follow(now);
      break;
    case FsmState::TASK_WAIT:
      compute_task_wait();
      break;
    case FsmState::FORMATION_ALIGN:
      compute_alignment(now);
      break;
  }

  RCLCPP_DEBUG(node_->get_logger(),
    "FSM state %d -> cmd_vel (%.3f, %.3f)",
    static_cast<int>(state_),
    cmd_vel_.linear.x,
    cmd_vel_.angular.z);
}

void SubController::compute_follow(const rclcpp::Time & /*now*/)
{
  const double main_yaw = get_yaw(last_odom_.pose.pose.orientation);
  const double desired_x = last_odom_.pose.pose.position.x - params_.target_distance * std::cos(main_yaw);
  const double desired_y = last_odom_.pose.pose.position.y - params_.target_distance * std::sin(main_yaw);
  const double desired_yaw = main_yaw;

  const double err_x = desired_x - est_x_;
  const double err_y = desired_y - est_y_;
  const double err_yaw = angles::shortest_angular_distance(est_yaw_, desired_yaw);

  const double cos_yaw = std::cos(est_yaw_);
  const double sin_yaw = std::sin(est_yaw_);
  const double err_body_x = cos_yaw * err_x + sin_yaw * err_y;
  const double err_body_y = -sin_yaw * err_x + cos_yaw * err_y;

  double lin_cmd = params_.k_lin * err_body_x;
  double ang_cmd = params_.k_lat * err_body_y + params_.k_yaw * err_yaw;

  cmd_vel_ = build_twist(lin_cmd, ang_cmd);
  RCLCPP_DEBUG(node_->get_logger(),
    "compute_follow -> err_body(%.3f, %.3f) yaw_err %.3f -> cmd (%.3f, %.3f)",
    err_body_x, err_body_y, err_yaw, cmd_vel_.linear.x, cmd_vel_.angular.z);
}

void SubController::compute_task_wait()
{
  // Simple exponential slowdown: blend previous command towards zero.
  const double decay = 0.2;
  const double lin_cmd = cmd_vel_.linear.x * (1.0 - decay);
  const double ang_cmd = cmd_vel_.angular.z * (1.0 - decay);
  cmd_vel_ = build_twist(lin_cmd, ang_cmd);
  if (std::fabs(cmd_vel_.linear.x) < 1e-3 && std::fabs(cmd_vel_.angular.z) < 1e-3) {
    cmd_vel_ = build_twist(0.0, 0.0);
  }
  RCLCPP_DEBUG(node_->get_logger(), "compute_task_wait -> cmd (%.3f, %.3f)", cmd_vel_.linear.x, cmd_vel_.angular.z);
}

void SubController::compute_alignment(const rclcpp::Time & /*now*/)
{
  const double main_yaw = get_yaw(last_odom_.pose.pose.orientation);
  const double desired_x = last_odom_.pose.pose.position.x - params_.align_offset_x * std::cos(main_yaw)
    + params_.align_offset_y * std::sin(main_yaw);
  const double desired_y = last_odom_.pose.pose.position.y - params_.align_offset_x * std::sin(main_yaw)
    - params_.align_offset_y * std::cos(main_yaw);
  const double desired_yaw = main_yaw;

  const double err_x = desired_x - est_x_;
  const double err_y = desired_y - est_y_;
  const double err_yaw = angles::shortest_angular_distance(est_yaw_, desired_yaw);

  const double cos_yaw = std::cos(est_yaw_);
  const double sin_yaw = std::sin(est_yaw_);
  const double err_body_x = cos_yaw * err_x + sin_yaw * err_y;
  const double err_body_y = -sin_yaw * err_x + cos_yaw * err_y;

  const double position_error = std::hypot(err_x, err_y);
  if (position_error < params_.tol_xy && std::fabs(err_yaw) < params_.tol_yaw) {
    RCLCPP_INFO(node_->get_logger(), "Alignment achieved -> FOLLOW");
    to_state(FsmState::FOLLOW);
    compute_follow(rclcpp::Time(0));
    return;
  }

  double lin_cmd = params_.k_lin * err_body_x;
  double ang_cmd = params_.k_lat * err_body_y + params_.k_yaw * err_yaw;

  cmd_vel_ = build_twist(lin_cmd, ang_cmd);
  RCLCPP_DEBUG(node_->get_logger(),
    "compute_alignment -> err(%.3f, %.3f, %.3f) -> cmd (%.3f, %.3f)",
    err_x, err_y, err_yaw, cmd_vel_.linear.x, cmd_vel_.angular.z);
}

void SubController::apply_stop()
{
  cmd_vel_ = build_twist(0.0, 0.0);
  RCLCPP_DEBUG(node_->get_logger(), "apply_stop -> zero velocity command issued");
}

void SubController::to_state(FsmState next_state)
{
  if (state_ == next_state) {
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "FSM transition %d -> %d", static_cast<int>(state_), static_cast<int>(next_state));
  state_ = next_state;
}

bool SubController::odom_stale(const rclcpp::Time & now) const
{
  if (!has_odom_) {
    return true;
  }
  const double dt = (now - last_odom_time_).seconds();
  return dt > params_.odom_timeout;
}

geometry_msgs::msg::Twist SubController::build_twist(double lin_x, double ang_z) const
{
  geometry_msgs::msg::Twist out;
  out.linear.x = clamp(lin_x, params_.max_lin);
  out.angular.z = clamp(ang_z, params_.max_ang);
  return out;
}

double SubController::clamp(double val, double limit) const
{
  if (val > limit) {
    return limit;
  }
  if (val < -limit) {
    return -limit;
  }
  return val;
}

double SubController::get_yaw(const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

void SubController::integrate_estimate(double dt)
{
  if (dt <= 0.0) {
    return;
  }
  const double vx = cmd_vel_.linear.x;
  const double wz = cmd_vel_.angular.z;
  est_x_ += vx * std::cos(est_yaw_) * dt;
  est_y_ += vx * std::sin(est_yaw_) * dt;
  est_yaw_ = angles::normalize_angle(est_yaw_ + wz * dt);
}

void SubController::initialise_estimate()
{
  const double main_yaw = get_yaw(last_odom_.pose.pose.orientation);
  est_x_ = last_odom_.pose.pose.position.x - params_.target_distance * std::cos(main_yaw);
  est_y_ = last_odom_.pose.pose.position.y - params_.target_distance * std::sin(main_yaw);
  est_yaw_ = main_yaw;
  estimate_initialised_ = true;
  last_update_time_ = last_odom_time_;
  RCLCPP_INFO(node_->get_logger(), "Initialised sub pose estimate at (%.2f, %.2f, %.2f)", est_x_, est_y_, est_yaw_);
}

}  // namespace sub_robot_core
