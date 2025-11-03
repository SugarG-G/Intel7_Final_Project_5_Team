#include <rclcpp/rclcpp.hpp>

#include "crop_task_vision/crop_task_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<crop_task_vision::CropTaskNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
