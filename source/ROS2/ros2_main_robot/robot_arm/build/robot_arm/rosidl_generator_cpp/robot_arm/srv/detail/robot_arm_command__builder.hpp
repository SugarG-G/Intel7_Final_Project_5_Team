// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_arm:srv/RobotArmCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__BUILDER_HPP_
#define ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_arm/srv/detail/robot_arm_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_arm
{

namespace srv
{

namespace builder
{

class Init_RobotArmCommand_Request_command
{
public:
  Init_RobotArmCommand_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_arm::srv::RobotArmCommand_Request command(::robot_arm::srv::RobotArmCommand_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_arm::srv::RobotArmCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_arm::srv::RobotArmCommand_Request>()
{
  return robot_arm::srv::builder::Init_RobotArmCommand_Request_command();
}

}  // namespace robot_arm


namespace robot_arm
{

namespace srv
{

namespace builder
{

class Init_RobotArmCommand_Response_result
{
public:
  Init_RobotArmCommand_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_arm::srv::RobotArmCommand_Response result(::robot_arm::srv::RobotArmCommand_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_arm::srv::RobotArmCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_arm::srv::RobotArmCommand_Response>()
{
  return robot_arm::srv::builder::Init_RobotArmCommand_Response_result();
}

}  // namespace robot_arm

#endif  // ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__BUILDER_HPP_
