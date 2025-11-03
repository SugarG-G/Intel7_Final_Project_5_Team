#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class MockMainOdomNode : public rclcpp::Node
{
public:
  MockMainOdomNode()
  : rclcpp::Node("mock_main_odom")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "main_base_link");
    linear_speed_ = declare_parameter<double>("linear_speed", 0.3);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.2);
    straight_duration_ = declare_parameter<double>("straight_duration", 10.0);
    loop_radius_ = declare_parameter<double>("loop_radius", 1.5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/main/odom", rclcpp::SensorDataQoS());

    start_time_ = now();
    timer_ = create_wall_timer(50ms, std::bind(&MockMainOdomNode::on_timer, this));
    RCLCPP_INFO(get_logger(),
      "mock_main_odom started (frame_id=%s, child_frame_id=%s, linear=%.2f, angular=%.2f)",
      frame_id_.c_str(),
      child_frame_id_.c_str(),
      linear_speed_,
      angular_speed_);
  }

private:
  void on_timer()
  {
    const auto current_time = now();
    const double t = (current_time - start_time_).seconds();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.child_frame_id = child_frame_id_;

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    geometry_msgs::msg::Twist twist;

    if (t < straight_duration_) {
      x = linear_speed_ * t;
      y = 0.0;
      yaw = 0.0;
      twist.linear.x = linear_speed_;
      twist.angular.z = 0.0;
    } else {
      const double t_loop = t - straight_duration_;
      const double angle = angular_speed_ * t_loop;
      x = linear_speed_ * straight_duration_ + loop_radius_ * std::cos(angle);
      y = loop_radius_ * std::sin(angle);
      yaw = angle + M_PI_2;
      twist.linear.x = loop_radius_ * angular_speed_;
      twist.angular.z = angular_speed_;
    }

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    odom_msg.twist.twist = twist;

    odom_pub_->publish(odom_msg);

    RCLCPP_DEBUG(get_logger(),
      "Published mock odom t=%.2f pose=(%.2f, %.2f, %.2f) twist=(%.2f, %.2f)",
      t,
      x,
      y,
      yaw,
      twist.linear.x,
      twist.angular.z);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  std::string child_frame_id_;
  double linear_speed_;
  double angular_speed_;
  double straight_duration_;
  double loop_radius_;

  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockMainOdomNode>());
  rclcpp::shutdown();
  return 0;
}
