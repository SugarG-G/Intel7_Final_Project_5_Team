#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "robot_arm/srv/robot_arm_command.hpp"
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <thread>
#include <chrono>

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
        uart_fd_ = open("/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF555071494867115341-if01",
                O_RDWR | O_NOCTTY | O_SYNC);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "UART open 실패: %s (%d)", strerror(errno), errno);
            return;
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
    bool write_all(int fd, const std::string &data)
    {
        const char* buf = data.c_str();
        size_t total = 0;
        size_t len = data.size();
        while (total < len) {
            ssize_t n = ::write(fd, buf + total, len - total);
            if (n < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            total += static_cast<size_t>(n);
        }
        // 전송 종료까지 대기
        if (tcdrain(fd) != 0) return false;
        return true;
    }

    bool send_uart_line(const std::string &line_no_newline)
    {
        // 개행은 전송 구분을 위해 추가.
        std::string payload = line_no_newline + "\n";
        return write_all(uart_fd_, payload);
    }

    void send_sequence(const std::vector<std::string> &seq, int delay_sec)
    {
        for (const auto &cmd : seq) {
            if (!send_uart_line(cmd)) {
                RCLCPP_ERROR(this->get_logger(), "UART 전송 실패: %s", cmd.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "아두이노로 전송됨: %s", cmd.c_str());
            }
            std::this_thread::sleep_for(std::chrono::seconds(delay_sec));
        }
    }

    void handle_command(
        const std::shared_ptr<RobotArmCommand::Request> request,
        std::shared_ptr<RobotArmCommand::Response> response)
    {
        const std::string command = request->command;
        RCLCPP_INFO(this->get_logger(), "수신 명령: %s", command.c_str());

        if (uart_fd_ < 0) {
            response->result = "UART 연결 안됨";
            return;
        }

        if (command == "detect") {
            const std::vector<std::string> seq = {
                "1500@1500@1500@1500@1500@1500", //믈체 주우러 가기
                "1500@1500@1800@1600@1700@1500", //물체 주우러 가기
                "600@1500@1800@1600@1800@1500", //집게 벌림
                "1700@1500@1800@1600@1800@1500", //물체 잡기
                "1700@1500@1700@1500@1600@1500", //놓기 준비
                "1700@1500@1800@1500@1600@500", //회전
                "1700@1500@2000@1500@1700@500", //물체 놓는 곳 위치
                "700@1500@2000@1500@1700@500", //믈체 놓기
                "1500@1500@1800@1500@1600@1500", //원위치 가는 길
                "1500@1500@1200@1500@1500@1500" //원위치
            };
            send_sequence(seq,2.5);
            response->result = "detect 시퀀스 전송 완료";
        } else if (command == "move") {
            const std::vector<std::string> seq = {
                // "1700@1500@1800@1500@1500@500",
                // "1700@1500@2000@1500@1700@500",
                // "700@1500@2000@1500@1700@500",
                // "1500@1500@1500@1500@1500@1500"
            };
            send_sequence(seq, 2);
            response->result = "move 시퀀스 전송 완료";
        } else {
            // 임의 문자열은 단일 명령으로 그대로 전송
            if (send_uart_line(command)) {
                response->result = "Command '" + command + "' sent successfully.";
            } else {
                response->result = "전송 오류";
            }
        }
    }

    void configure_uart()
    {
        struct termios tty;
        if (tcgetattr(uart_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "UART 속성 설정 실패");
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

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
    int uart_fd_ = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

