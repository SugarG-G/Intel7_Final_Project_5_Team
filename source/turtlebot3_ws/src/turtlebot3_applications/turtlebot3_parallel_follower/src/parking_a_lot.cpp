#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // In the future, a node will be created and spun here.
  rclcpp::shutdown();
  return 0;
}
