// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_applications_msgs:srv/SetFollowState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__TRAITS_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_applications_msgs/srv/detail/set_follow_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_applications_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetFollowState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetFollowState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetFollowState_Request & msg, bool use_flow_style = false)
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

}  // namespace turtlebot3_applications_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_applications_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_applications_msgs::srv::SetFollowState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_applications_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_applications_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_applications_msgs::srv::SetFollowState_Request & msg)
{
  return turtlebot3_applications_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_applications_msgs::srv::SetFollowState_Request>()
{
  return "turtlebot3_applications_msgs::srv::SetFollowState_Request";
}

template<>
inline const char * name<turtlebot3_applications_msgs::srv::SetFollowState_Request>()
{
  return "turtlebot3_applications_msgs/srv/SetFollowState_Request";
}

template<>
struct has_fixed_size<turtlebot3_applications_msgs::srv::SetFollowState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_applications_msgs::srv::SetFollowState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_applications_msgs::srv::SetFollowState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_applications_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetFollowState_Response & msg,
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
  const SetFollowState_Response & msg,
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

inline std::string to_yaml(const SetFollowState_Response & msg, bool use_flow_style = false)
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

}  // namespace turtlebot3_applications_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_applications_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_applications_msgs::srv::SetFollowState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_applications_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_applications_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_applications_msgs::srv::SetFollowState_Response & msg)
{
  return turtlebot3_applications_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_applications_msgs::srv::SetFollowState_Response>()
{
  return "turtlebot3_applications_msgs::srv::SetFollowState_Response";
}

template<>
inline const char * name<turtlebot3_applications_msgs::srv::SetFollowState_Response>()
{
  return "turtlebot3_applications_msgs/srv/SetFollowState_Response";
}

template<>
struct has_fixed_size<turtlebot3_applications_msgs::srv::SetFollowState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_applications_msgs::srv::SetFollowState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_applications_msgs::srv::SetFollowState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_applications_msgs::srv::SetFollowState>()
{
  return "turtlebot3_applications_msgs::srv::SetFollowState";
}

template<>
inline const char * name<turtlebot3_applications_msgs::srv::SetFollowState>()
{
  return "turtlebot3_applications_msgs/srv/SetFollowState";
}

template<>
struct has_fixed_size<turtlebot3_applications_msgs::srv::SetFollowState>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_applications_msgs::srv::SetFollowState_Request>::value &&
    has_fixed_size<turtlebot3_applications_msgs::srv::SetFollowState_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_applications_msgs::srv::SetFollowState>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_applications_msgs::srv::SetFollowState_Request>::value &&
    has_bounded_size<turtlebot3_applications_msgs::srv::SetFollowState_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_applications_msgs::srv::SetFollowState>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_applications_msgs::srv::SetFollowState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_applications_msgs::srv::SetFollowState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__TRAITS_HPP_
