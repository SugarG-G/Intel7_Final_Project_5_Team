#ifndef CROP_TASK_VISION__LIDAR_GUARD_NODE_HPP_
#define CROP_TASK_VISION__LIDAR_GUARD_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace crop_task_vision
{

class LidarGuardNode : public rclcpp::Node
{
public:
  explicit LidarGuardNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void configure_parameters();
  void initialise_interfaces();
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publish_event(const std::string & text);
  void publish_stop(bool flag);

  double compute_front_min_range(const sensor_msgs::msg::LaserScan & scan) const;

  // Parameters
  std::string scan_topic_;
  std::string event_topic_;
  std::string stop_topic_;
  bool publish_stop_{false};
  double trigger_distance_{0.45};
  double release_distance_{0.55};
  double front_window_deg_{70.0};
  int required_hits_{2};
  int required_clear_{6};

  // State
  bool obstacle_active_{false};
  int hit_counter_{0};
  int clear_counter_{0};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
};

}  // namespace crop_task_vision

#endif  // CROP_TASK_VISION__LIDAR_GUARD_NODE_HPP_

