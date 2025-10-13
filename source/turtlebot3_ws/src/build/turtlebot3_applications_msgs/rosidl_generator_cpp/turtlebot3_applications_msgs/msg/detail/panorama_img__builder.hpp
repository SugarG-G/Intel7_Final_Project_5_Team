// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_applications_msgs:msg/PanoramaImg.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__BUILDER_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_applications_msgs/msg/detail/panorama_img__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_applications_msgs
{

namespace msg
{

namespace builder
{

class Init_PanoramaImg_image
{
public:
  explicit Init_PanoramaImg_image(::turtlebot3_applications_msgs::msg::PanoramaImg & msg)
  : msg_(msg)
  {}
  ::turtlebot3_applications_msgs::msg::PanoramaImg image(::turtlebot3_applications_msgs::msg::PanoramaImg::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_applications_msgs::msg::PanoramaImg msg_;
};

class Init_PanoramaImg_geo_tag
{
public:
  explicit Init_PanoramaImg_geo_tag(::turtlebot3_applications_msgs::msg::PanoramaImg & msg)
  : msg_(msg)
  {}
  Init_PanoramaImg_image geo_tag(::turtlebot3_applications_msgs::msg::PanoramaImg::_geo_tag_type arg)
  {
    msg_.geo_tag = std::move(arg);
    return Init_PanoramaImg_image(msg_);
  }

private:
  ::turtlebot3_applications_msgs::msg::PanoramaImg msg_;
};

class Init_PanoramaImg_heading
{
public:
  explicit Init_PanoramaImg_heading(::turtlebot3_applications_msgs::msg::PanoramaImg & msg)
  : msg_(msg)
  {}
  Init_PanoramaImg_geo_tag heading(::turtlebot3_applications_msgs::msg::PanoramaImg::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_PanoramaImg_geo_tag(msg_);
  }

private:
  ::turtlebot3_applications_msgs::msg::PanoramaImg msg_;
};

class Init_PanoramaImg_longitude
{
public:
  explicit Init_PanoramaImg_longitude(::turtlebot3_applications_msgs::msg::PanoramaImg & msg)
  : msg_(msg)
  {}
  Init_PanoramaImg_heading longitude(::turtlebot3_applications_msgs::msg::PanoramaImg::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_PanoramaImg_heading(msg_);
  }

private:
  ::turtlebot3_applications_msgs::msg::PanoramaImg msg_;
};

class Init_PanoramaImg_latitude
{
public:
  explicit Init_PanoramaImg_latitude(::turtlebot3_applications_msgs::msg::PanoramaImg & msg)
  : msg_(msg)
  {}
  Init_PanoramaImg_longitude latitude(::turtlebot3_applications_msgs::msg::PanoramaImg::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_PanoramaImg_longitude(msg_);
  }

private:
  ::turtlebot3_applications_msgs::msg::PanoramaImg msg_;
};

class Init_PanoramaImg_pano_id
{
public:
  explicit Init_PanoramaImg_pano_id(::turtlebot3_applications_msgs::msg::PanoramaImg & msg)
  : msg_(msg)
  {}
  Init_PanoramaImg_latitude pano_id(::turtlebot3_applications_msgs::msg::PanoramaImg::_pano_id_type arg)
  {
    msg_.pano_id = std::move(arg);
    return Init_PanoramaImg_latitude(msg_);
  }

private:
  ::turtlebot3_applications_msgs::msg::PanoramaImg msg_;
};

class Init_PanoramaImg_header
{
public:
  Init_PanoramaImg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PanoramaImg_pano_id header(::turtlebot3_applications_msgs::msg::PanoramaImg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PanoramaImg_pano_id(msg_);
  }

private:
  ::turtlebot3_applications_msgs::msg::PanoramaImg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_applications_msgs::msg::PanoramaImg>()
{
  return turtlebot3_applications_msgs::msg::builder::Init_PanoramaImg_header();
}

}  // namespace turtlebot3_applications_msgs

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__BUILDER_HPP_
