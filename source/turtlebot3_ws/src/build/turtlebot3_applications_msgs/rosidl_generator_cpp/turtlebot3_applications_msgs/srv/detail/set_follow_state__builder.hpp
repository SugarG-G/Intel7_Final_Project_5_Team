// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_applications_msgs:srv/SetFollowState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__BUILDER_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_applications_msgs/srv/detail/set_follow_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_applications_msgs
{

namespace srv
{

namespace builder
{

class Init_SetFollowState_Request_state
{
public:
  Init_SetFollowState_Request_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_applications_msgs::srv::SetFollowState_Request state(::turtlebot3_applications_msgs::srv::SetFollowState_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_applications_msgs::srv::SetFollowState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_applications_msgs::srv::SetFollowState_Request>()
{
  return turtlebot3_applications_msgs::srv::builder::Init_SetFollowState_Request_state();
}

}  // namespace turtlebot3_applications_msgs


namespace turtlebot3_applications_msgs
{

namespace srv
{

namespace builder
{

class Init_SetFollowState_Response_result
{
public:
  Init_SetFollowState_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_applications_msgs::srv::SetFollowState_Response result(::turtlebot3_applications_msgs::srv::SetFollowState_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_applications_msgs::srv::SetFollowState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_applications_msgs::srv::SetFollowState_Response>()
{
  return turtlebot3_applications_msgs::srv::builder::Init_SetFollowState_Response_result();
}

}  // namespace turtlebot3_applications_msgs

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__BUILDER_HPP_
