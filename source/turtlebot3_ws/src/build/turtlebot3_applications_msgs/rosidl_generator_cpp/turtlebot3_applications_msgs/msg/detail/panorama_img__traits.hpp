// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_applications_msgs:msg/PanoramaImg.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__TRAITS_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_applications_msgs/msg/detail/panorama_img__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace turtlebot3_applications_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PanoramaImg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pano_id
  {
    out << "pano_id: ";
    rosidl_generator_traits::value_to_yaml(msg.pano_id, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: geo_tag
  {
    out << "geo_tag: ";
    rosidl_generator_traits::value_to_yaml(msg.geo_tag, out);
    out << ", ";
  }

  // member: image
  {
    out << "image: ";
    to_flow_style_yaml(msg.image, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PanoramaImg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: pano_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pano_id: ";
    rosidl_generator_traits::value_to_yaml(msg.pano_id, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: geo_tag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geo_tag: ";
    rosidl_generator_traits::value_to_yaml(msg.geo_tag, out);
    out << "\n";
  }

  // member: image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image:\n";
    to_block_style_yaml(msg.image, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PanoramaImg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace turtlebot3_applications_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_applications_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_applications_msgs::msg::PanoramaImg & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_applications_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_applications_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_applications_msgs::msg::PanoramaImg & msg)
{
  return turtlebot3_applications_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_applications_msgs::msg::PanoramaImg>()
{
  return "turtlebot3_applications_msgs::msg::PanoramaImg";
}

template<>
inline const char * name<turtlebot3_applications_msgs::msg::PanoramaImg>()
{
  return "turtlebot3_applications_msgs/msg/PanoramaImg";
}

template<>
struct has_fixed_size<turtlebot3_applications_msgs::msg::PanoramaImg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_applications_msgs::msg::PanoramaImg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_applications_msgs::msg::PanoramaImg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__TRAITS_HPP_
