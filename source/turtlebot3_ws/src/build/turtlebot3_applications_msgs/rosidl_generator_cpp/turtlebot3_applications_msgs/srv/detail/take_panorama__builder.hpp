// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_applications_msgs:srv/TakePanorama.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__BUILDER_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_applications_msgs/srv/detail/take_panorama__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_applications_msgs
{

namespace srv
{

namespace builder
{

class Init_TakePanorama_Request_rot_vel
{
public:
  explicit Init_TakePanorama_Request_rot_vel(::turtlebot3_applications_msgs::srv::TakePanorama_Request & msg)
  : msg_(msg)
  {}
  ::turtlebot3_applications_msgs::srv::TakePanorama_Request rot_vel(::turtlebot3_applications_msgs::srv::TakePanorama_Request::_rot_vel_type arg)
  {
    msg_.rot_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_applications_msgs::srv::TakePanorama_Request msg_;
};

class Init_TakePanorama_Request_snap_interval
{
public:
  explicit Init_TakePanorama_Request_snap_interval(::turtlebot3_applications_msgs::srv::TakePanorama_Request & msg)
  : msg_(msg)
  {}
  Init_TakePanorama_Request_rot_vel snap_interval(::turtlebot3_applications_msgs::srv::TakePanorama_Request::_snap_interval_type arg)
  {
    msg_.snap_interval = std::move(arg);
    return Init_TakePanorama_Request_rot_vel(msg_);
  }

private:
  ::turtlebot3_applications_msgs::srv::TakePanorama_Request msg_;
};

class Init_TakePanorama_Request_pano_angle
{
public:
  explicit Init_TakePanorama_Request_pano_angle(::turtlebot3_applications_msgs::srv::TakePanorama_Request & msg)
  : msg_(msg)
  {}
  Init_TakePanorama_Request_snap_interval pano_angle(::turtlebot3_applications_msgs::srv::TakePanorama_Request::_pano_angle_type arg)
  {
    msg_.pano_angle = std::move(arg);
    return Init_TakePanorama_Request_snap_interval(msg_);
  }

private:
  ::turtlebot3_applications_msgs::srv::TakePanorama_Request msg_;
};

class Init_TakePanorama_Request_mode
{
public:
  Init_TakePanorama_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TakePanorama_Request_pano_angle mode(::turtlebot3_applications_msgs::srv::TakePanorama_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_TakePanorama_Request_pano_angle(msg_);
  }

private:
  ::turtlebot3_applications_msgs::srv::TakePanorama_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_applications_msgs::srv::TakePanorama_Request>()
{
  return turtlebot3_applications_msgs::srv::builder::Init_TakePanorama_Request_mode();
}

}  // namespace turtlebot3_applications_msgs


namespace turtlebot3_applications_msgs
{

namespace srv
{

namespace builder
{

class Init_TakePanorama_Response_status
{
public:
  Init_TakePanorama_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_applications_msgs::srv::TakePanorama_Response status(::turtlebot3_applications_msgs::srv::TakePanorama_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_applications_msgs::srv::TakePanorama_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_applications_msgs::srv::TakePanorama_Response>()
{
  return turtlebot3_applications_msgs::srv::builder::Init_TakePanorama_Response_status();
}

}  // namespace turtlebot3_applications_msgs

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__BUILDER_HPP_
