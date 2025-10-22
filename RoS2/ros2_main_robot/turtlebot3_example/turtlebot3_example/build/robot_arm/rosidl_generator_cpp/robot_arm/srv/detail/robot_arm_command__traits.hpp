// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_arm:srv/RobotArmCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__TRAITS_HPP_
#define ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_arm/srv/detail/robot_arm_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_arm
{

namespace srv
{

inline void to_flow_style_yaml(
  const RobotArmCommand_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotArmCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotArmCommand_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_arm

namespace rosidl_generator_traits
{

[[deprecated("use robot_arm::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_arm::srv::RobotArmCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_arm::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_arm::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_arm::srv::RobotArmCommand_Request & msg)
{
  return robot_arm::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_arm::srv::RobotArmCommand_Request>()
{
  return "robot_arm::srv::RobotArmCommand_Request";
}

template<>
inline const char * name<robot_arm::srv::RobotArmCommand_Request>()
{
  return "robot_arm/srv/RobotArmCommand_Request";
}

template<>
struct has_fixed_size<robot_arm::srv::RobotArmCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_arm::srv::RobotArmCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_arm::srv::RobotArmCommand_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_arm
{

namespace srv
{

inline void to_flow_style_yaml(
  const RobotArmCommand_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotArmCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotArmCommand_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_arm

namespace rosidl_generator_traits
{

[[deprecated("use robot_arm::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_arm::srv::RobotArmCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_arm::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_arm::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_arm::srv::RobotArmCommand_Response & msg)
{
  return robot_arm::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_arm::srv::RobotArmCommand_Response>()
{
  return "robot_arm::srv::RobotArmCommand_Response";
}

template<>
inline const char * name<robot_arm::srv::RobotArmCommand_Response>()
{
  return "robot_arm/srv/RobotArmCommand_Response";
}

template<>
struct has_fixed_size<robot_arm::srv::RobotArmCommand_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_arm::srv::RobotArmCommand_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_arm::srv::RobotArmCommand_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_arm::srv::RobotArmCommand>()
{
  return "robot_arm::srv::RobotArmCommand";
}

template<>
inline const char * name<robot_arm::srv::RobotArmCommand>()
{
  return "robot_arm/srv/RobotArmCommand";
}

template<>
struct has_fixed_size<robot_arm::srv::RobotArmCommand>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_arm::srv::RobotArmCommand_Request>::value &&
    has_fixed_size<robot_arm::srv::RobotArmCommand_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_arm::srv::RobotArmCommand>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_arm::srv::RobotArmCommand_Request>::value &&
    has_bounded_size<robot_arm::srv::RobotArmCommand_Response>::value
  >
{
};

template<>
struct is_service<robot_arm::srv::RobotArmCommand>
  : std::true_type
{
};

template<>
struct is_service_request<robot_arm::srv::RobotArmCommand_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_arm::srv::RobotArmCommand_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__TRAITS_HPP_
