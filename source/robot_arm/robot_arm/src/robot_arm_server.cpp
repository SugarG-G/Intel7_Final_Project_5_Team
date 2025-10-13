#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "robot_arm/srv/robot_arm_command.hpp"
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using RobotArmCommand = robot_arm::srv::RobotArmCommand;

class RobotArmServiceServer : public rclcpp::Node
{
public:
    RobotArmServiceServer()
    : Node("robot_arm_service_server")
    {
        service_ = this->create_service<RobotArmCommand>(
            "robot_arm_control",
            std::bind(&RobotArmServiceServer::handle_command, this, std::placeholders::_1, std::placeholders::_2)
        );

        // UART 설정
        uart_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "UART 연결 실패");
        } else {
            configure_uart();
            RCLCPP_INFO(this->get_logger(), "UART 연결 완료, 서비스 준비됨.");
        }
    }

    ~RobotArmServiceServer()
    {
        if (uart_fd_ >= 0) close(uart_fd_);
    }

private:
    void handle_command(
        const std::shared_ptr<RobotArmCommand::Request> request,
        std::shared_ptr<RobotArmCommand::Response> response)
    {
        std::string command = request->command;
        RCLCPP_INFO(this->get_logger(), "수신 명령: %s", command.c_str());

        if (uart_fd_ >= 0) {
            std::string cmd_with_newline = command + "\n";
            ssize_t n = write(uart_fd_, cmd_with_newline.c_str(), cmd_with_newline.size());
            if (n < 0) {
                response->result = "전송 오류";
                RCLCPP_ERROR(this->get_logger(), "UART 전송 실패");
            } else {
                response->result = "Command '" + command + "' sent successfully.";
                RCLCPP_INFO(this->get_logger(), "아두이노로 전송됨: %s", command.c_str());
            }
        } else {
            response->result = "UART 연결 안됨";
        }
    }

    void configure_uart()
    {
        struct termios tty;
        if (tcgetattr(uart_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "UART 속성 설정 실패");
            return;
        }

        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8비트
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 1;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tcsetattr(uart_fd_, TCSANOW, &tty);
    }

    rclcpp::Service<RobotArmCommand>::SharedPtr service_;
    int uart_fd_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

