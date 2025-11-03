#include "sub_robot_core/sub_controller.hpp"

#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr auto kOdomTopic = "/main/odom";
constexpr auto kCommandTopic = "/main/task_simple";
constexpr auto kCmdVelTopic = "/sub/cmd_vel";
constexpr double kControlFrequency = 50.0;  // Hz
}  // namespace

class SubControllerNode : public rclcpp::Node
{
public:
  SubControllerNode()
  : rclcpp::Node("sub_controller_node"), controller_(this)
  {
    using std::placeholders::_1;

    RCLCPP_INFO(this->get_logger(), "sub_controller_node starting up");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      kOdomTopic, rclcpp::SensorDataQoS(), std::bind(&SubControllerNode::on_odom, this, _1));

    command_sub_ = create_subscription<std_msgs::msg::UInt8>(
      kCommandTopic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SubControllerNode::on_command, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);

    const auto period = std::chrono::duration<double>(1.0 / kControlFrequency);
    control_timer_ = create_wall_timer(period, std::bind(&SubControllerNode::on_timer, this));
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    controller_.handle_odom(msg);
    RCLCPP_DEBUG(get_logger(), "Odom callback processed");
  }

  void on_command(const std_msgs::msg::UInt8 & msg)
  {
    controller_.handle_command(msg.data);
    RCLCPP_DEBUG(get_logger(), "Command callback processed");
  }

  void on_timer()
  {
    const auto now = get_clock()->now();
    controller_.update(now);
    cmd_pub_->publish(controller_.current_cmd());
    RCLCPP_DEBUG(get_logger(), "Published cmd_vel" );
  }

  sub_robot_core::SubController controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubControllerNode>());
  rclcpp::shutdown();
  return 0;
}
