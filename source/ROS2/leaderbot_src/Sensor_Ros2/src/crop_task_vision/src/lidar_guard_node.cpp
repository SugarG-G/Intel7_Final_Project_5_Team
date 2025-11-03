#include "crop_task_vision/lidar_guard_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace crop_task_vision
{

LidarGuardNode::LidarGuardNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lidar_guard_node", options)
{
  configure_parameters();
  initialise_interfaces();

  RCLCPP_INFO(
    get_logger(),
    "LiDAR guard ready. Watching %s (trigger %.2fm, release %.2fm, window %.1f°)",
    scan_topic_.c_str(), trigger_distance_, release_distance_, front_window_deg_);
}

void LidarGuardNode::configure_parameters()
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  event_topic_ = declare_parameter<std::string>("event_topic", "crop_task/events");
  stop_topic_ = declare_parameter<std::string>("stop_topic", "patrol/stop");
  publish_stop_ = declare_parameter<bool>("publish_stop", publish_stop_);
  trigger_distance_ = declare_parameter<double>("trigger_distance", trigger_distance_);
  release_distance_ = declare_parameter<double>("release_distance", release_distance_);
  front_window_deg_ = declare_parameter<double>("front_window_deg", front_window_deg_);
  required_hits_ = declare_parameter<int>("required_hits", required_hits_);
  required_clear_ = declare_parameter<int>("required_clear", required_clear_);

  if (trigger_distance_ <= 0.0) {
    trigger_distance_ = 0.2;
  }
  if (release_distance_ <= trigger_distance_) {
    release_distance_ = trigger_distance_ + 0.1;
  }
  if (front_window_deg_ <= 0.0) {
    front_window_deg_ = 45.0;
  }
  required_hits_ = std::max(1, required_hits_);
  required_clear_ = std::max(1, required_clear_);
}

void LidarGuardNode::initialise_interfaces()
{
  using std::placeholders::_1;

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&LidarGuardNode::on_scan, this, _1));

  if (!event_topic_.empty()) {
    event_pub_ = create_publisher<std_msgs::msg::String>(event_topic_, rclcpp::QoS(10).reliable());
  }

  if (publish_stop_ && !stop_topic_.empty()) {
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic_, rclcpp::QoS(10).reliable());
  }
}

void LidarGuardNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const double min_range = compute_front_min_range(*msg);
  if (!std::isfinite(min_range)) {
    return;
  }

  if (!obstacle_active_) {
    if (min_range <= trigger_distance_) {
      hit_counter_++;
      if (hit_counter_ >= required_hits_) {
        obstacle_active_ = true;
        clear_counter_ = 0;
        publish_event("lidar_obstacle_detected:" + std::to_string(min_range));
        publish_stop(true);
        RCLCPP_WARN(
          get_logger(),
          "Obstacle detected at %.3fm. Stop requested.", min_range);
      }
    } else {
      hit_counter_ = 0;
    }
  } else {
    if (min_range >= release_distance_) {
      clear_counter_++;
      if (clear_counter_ >= required_clear_) {
        obstacle_active_ = false;
        hit_counter_ = 0;
        publish_event("lidar_clear");
        publish_stop(false);
        RCLCPP_INFO(get_logger(), "Obstacle cleared (%.3fm).", min_range);
      }
    } else {
      clear_counter_ = 0;
    }
  }
}

double LidarGuardNode::compute_front_min_range(const sensor_msgs::msg::LaserScan & scan) const
{
  const double desired_half = front_window_deg_ * M_PI / 180.0 / 2.0;

  const double angle_min = scan.angle_min;
  const double angle_max = scan.angle_max;
  const double angle_increment = scan.angle_increment;

  if (angle_increment <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const int total_samples = static_cast<int>((angle_max - angle_min) / angle_increment);
  if (total_samples <= 0) {
    return std::numeric_limits<double>::infinity();
  }

  int start_idx = static_cast<int>((-desired_half - angle_min) / angle_increment);
  int end_idx = static_cast<int>((desired_half - angle_min) / angle_increment);

  start_idx = std::max(0, start_idx);
  end_idx = std::min(static_cast<int>(scan.ranges.size()) - 1, end_idx);

  double min_range = std::numeric_limits<double>::infinity();
  for (int i = start_idx; i <= end_idx; ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }
    if (range < min_range) {
      min_range = range;
    }
  }

  return min_range;
}

void LidarGuardNode::publish_event(const std::string & text)
{
  if (!event_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = text;
  event_pub_->publish(msg);
}

void LidarGuardNode::publish_stop(bool flag)
{
  if (!stop_pub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = flag;
  stop_pub_->publish(msg);
}

}  // namespace crop_task_vision

