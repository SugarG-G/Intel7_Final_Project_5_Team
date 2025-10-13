#include "rclcpp/rclcpp.hpp"
#include "robot_arm/srv/robot_arm_command.hpp"
#include <memory>
#include <string>
#include <iostream>

using RobotArmCommand = robot_arm::srv::RobotArmCommand;
using namespace std::chrono_literals;

class RobotArmClient : public rclcpp::Node
{
public:
    RobotArmClient()
    : Node("robot_arm_service_client")
    {
        client_ = this->create_client<RobotArmCommand>("robot_arm_control");

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "서비스 대기 중...");
        }
    }

    std::shared_ptr<RobotArmCommand::Response> send_command(const std::string & command)
    {
        auto request = std::make_shared<RobotArmCommand::Request>();
        request->command = command;

        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "서비스 호출 실패");
            return nullptr;
        }
    }

private:
    rclcpp::Client<RobotArmCommand>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmClient>();

    std::string cmd;
    std::cout << "보낼 명령 입력 (예: 70@90@40@90@0@90): ";
    std::getline(std::cin, cmd);

    auto response = node->send_command(cmd);
    if (response) {
        std::cout << "서버 응답: " << response->result << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}

