// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_applications_msgs:srv/TakePanorama.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__TRAITS_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_applications_msgs/srv/detail/take_panorama__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_applications_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const TakePanorama_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: pano_angle
  {
    out << "pano_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pano_angle, out);
    out << ", ";
  }

  // member: snap_interval
  {
    out << "snap_interval: ";
    rosidl_generator_traits::value_to_yaml(msg.snap_interval, out);
    out << ", ";
  }

  // member: rot_vel
  {
    out << "rot_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.rot_vel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TakePanorama_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: pano_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pano_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pano_angle, out);
    out << "\n";
  }

  // member: snap_interval
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "snap_interval: ";
    rosidl_generator_traits::value_to_yaml(msg.snap_interval, out);
    out << "\n";
  }

  // member: rot_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rot_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.rot_vel, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TakePanorama_Request & msg, bool use_flow_style = false)
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
  const turtlebot3_applications_msgs::srv::TakePanorama_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_applications_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_applications_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_applications_msgs::srv::TakePanorama_Request & msg)
{
  return turtlebot3_applications_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_applications_msgs::srv::TakePanorama_Request>()
{
  return "turtlebot3_applications_msgs::srv::TakePanorama_Request";
}

template<>
inline const char * name<turtlebot3_applications_msgs::srv::TakePanorama_Request>()
{
  return "turtlebot3_applications_msgs/srv/TakePanorama_Request";
}

template<>
struct has_fixed_size<turtlebot3_applications_msgs::srv::TakePanorama_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_applications_msgs::srv::TakePanorama_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_applications_msgs::srv::TakePanorama_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_applications_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const TakePanorama_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TakePanorama_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TakePanorama_Response & msg, bool use_flow_style = false)
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
  const turtlebot3_applications_msgs::srv::TakePanorama_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_applications_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_applications_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_applications_msgs::srv::TakePanorama_Response & msg)
{
  return turtlebot3_applications_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_applications_msgs::srv::TakePanorama_Response>()
{
  return "turtlebot3_applications_msgs::srv::TakePanorama_Response";
}

template<>
inline const char * name<turtlebot3_applications_msgs::srv::TakePanorama_Response>()
{
  return "turtlebot3_applications_msgs/srv/TakePanorama_Response";
}

template<>
struct has_fixed_size<turtlebot3_applications_msgs::srv::TakePanorama_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_applications_msgs::srv::TakePanorama_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_applications_msgs::srv::TakePanorama_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_applications_msgs::srv::TakePanorama>()
{
  return "turtlebot3_applications_msgs::srv::TakePanorama";
}

template<>
inline const char * name<turtlebot3_applications_msgs::srv::TakePanorama>()
{
  return "turtlebot3_applications_msgs/srv/TakePanorama";
}

template<>
struct has_fixed_size<turtlebot3_applications_msgs::srv::TakePanorama>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_applications_msgs::srv::TakePanorama_Request>::value &&
    has_fixed_size<turtlebot3_applications_msgs::srv::TakePanorama_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_applications_msgs::srv::TakePanorama>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_applications_msgs::srv::TakePanorama_Request>::value &&
    has_bounded_size<turtlebot3_applications_msgs::srv::TakePanorama_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_applications_msgs::srv::TakePanorama>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_applications_msgs::srv::TakePanorama_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_applications_msgs::srv::TakePanorama_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__TRAITS_HPP_
