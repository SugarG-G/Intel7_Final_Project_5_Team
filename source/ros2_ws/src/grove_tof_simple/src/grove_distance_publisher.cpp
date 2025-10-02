// basic c++ header files
#include <memory>
#include <string>
#include <chrono>

// for serial port
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>

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
    //Q. this name "publisher_" is used in ROS2 network? then this naming doesn't seems to be good.
    //A. this name is used only inside this class. so it's OK.
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("distance", 10);

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&GroveDistancePublisher::timer_callback, this)
        //Q. can't I write as "1s" directly here? isn't it chorno_literals feature?
        //A. you can write as "1s" directly here if you include "using namespace std::chrono_literals;" at the top of the file.
        //Q. why std::bind is needed here? Isn't lambda() more modern?
        //A. because we need to pass "this" pointer to member function
        //A. lambda can be used, but std::bind is more readable in this case
        //와 코파일럿이 답도 해줌, 근데 이 답이 맞는지는 잘 모르겠음
    );
  }

  ~GroveDistancePublisher()
  {

  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32MultiArray();

    message.data.resize(4);
    //Q. why not request, it's resize?
    //A. because Float32MultiArray is not a vector, it's a message type

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
  //Q. what is this long type?
  //A. it's a shared pointer to a publisher of Float32MultiArray message type
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroveDistancePublisher>());
    //Q. what is spin?
    //A. spin() is a function that keeps the node alive and processing callbacks(blocking)
    rclcpp::shutdown();
    //Q. why shutdown is here?
    //A. shutdown() is called to clean up resources before exiting the program
    //Q. My knowledge: this is called by ^c, right?
    return 0;
}
