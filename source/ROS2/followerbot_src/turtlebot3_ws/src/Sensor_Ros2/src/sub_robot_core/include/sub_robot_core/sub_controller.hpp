#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace sub_robot_core
{

enum class FsmState
{
  STOP = 0,
  FOLLOW = 1,
  TASK_WAIT = 2,
  FORMATION_ALIGN = 3,
};

struct ControllerParams
{
  double target_distance{0.6};
  double align_offset_y{0.5};
  double align_offset_x{0.1};
  double tol_xy{0.05};
  double tol_yaw{0.05};
  double max_lin{0.3};
  double max_ang{0.8};
  double odom_timeout{0.5};
  double k_lin{1.0};
  double k_lat{1.0};
  double k_yaw{1.5};
};

class SubController
{
public:
  explicit SubController(rclcpp::Node * node);

  void update(const rclcpp::Time & now);
  void handle_odom(const nav_msgs::msg::Odometry & msg);
  void handle_command(uint8_t command);

  geometry_msgs::msg::Twist current_cmd() const { return cmd_vel_; }
  FsmState current_state() const { return state_; }

private:
  void to_state(FsmState next_state);
  void compute_follow(const rclcpp::Time & now);
  void compute_task_wait();
  void compute_alignment(const rclcpp::Time & now);
  void apply_stop();
  bool odom_stale(const rclcpp::Time & now) const;

  geometry_msgs::msg::Twist build_twist(double lin_x, double ang_z) const;
  double clamp(double val, double limit) const;
  double get_yaw(const geometry_msgs::msg::Quaternion & q) const;
  void integrate_estimate(double dt);
  void initialise_estimate();

  rclcpp::Node * node_;
  ControllerParams params_;

  FsmState state_{FsmState::STOP};
  nav_msgs::msg::Odometry last_odom_{};
  bool has_odom_{false};
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_update_time_;
  geometry_msgs::msg::Twist cmd_vel_;

  // Simple internal pose estimate for the sub robot (x, y, yaw)
  bool estimate_initialised_{false};
  double est_x_{0.0};
  double est_y_{0.0};
  double est_yaw_{0.0};
};

}  // namespace sub_robot_core
