#include <rclcpp/rclcpp.hpp>

#include "crop_task_vision/lidar_guard_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<crop_task_vision::LidarGuardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
