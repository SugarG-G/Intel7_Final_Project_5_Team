#include <memory>
#include <chrono>

// ROS2 node, range message
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class GroveDistanceSubscriber : public rclcpp::Node
{
public:
  GroveDistanceSubscriber()
  : Node("grove_distance_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "distance", 10, std::bind(&GroveDistanceSubscriber::topic_callback, this, _1)); 
  } 

  private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received distances: [%.2f, %.2f, %.2f, %.2f]",
        msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroveDistanceSubscriber>());
  rclcpp::shutdown();
  return 0;
}
