// basic c++ header files
#include <memory>  // std::shared_ptr
#include <chrono>  // std::chrono_literals

// for serial port
#if 0
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#endif

// ROS2 node, range message
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class GroveDistancePublisher : public rclcpp::Node
{
public:
  GroveDistancePublisher()
  : Node("grove_distance_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("distance", 10);

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&GroveDistancePublisher::timer_callback, this)
    );
  }

  ~GroveDistancePublisher()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down grove_distance_publisher node.");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32MultiArray();

    message.data.resize(4);

    //TODO: these are just some random values, replace with actual sensor readings
    message.data[0] = static_cast<float>(rand() % 4000) / 100.0; // Simulated distance in cm
    message.data[1] = static_cast<float>(rand() % 4000) / 100.0; // Simulated distance in cm
    message.data[2] = static_cast<float>(rand() % 4000) / 100.0; // Simulated distance in cm
    message.data[3] = static_cast<float>(rand() % 4000) / 100.0; // Simulated distance in cm

    RCLCPP_INFO(this->get_logger(), "Publishing: [%.2f, %.2f, %.2f, %.2f]",
        message.data[0], message.data[1], message.data[2], message.data[3]);

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroveDistancePublisher>());
    rclcpp::shutdown();
    return 0;
}
