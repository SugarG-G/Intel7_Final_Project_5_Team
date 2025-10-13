// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_applications_msgs:srv/SetFollowState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__STRUCT_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Request __declspec(deprecated)
#endif

namespace turtlebot3_applications_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetFollowState_Request_
{
  using Type = SetFollowState_Request_<ContainerAllocator>;

  explicit SetFollowState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit SetFollowState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _state_type =
    uint8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STOPPED =
    0u;
  static constexpr uint8_t FOLLOW =
    1u;
  static constexpr uint8_t OK =
    0u;
  // guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
  static constexpr uint8_t ERROR =
    1u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif

  // pointer types
  using RawPtr =
    turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Request
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Request
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetFollowState_Request_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetFollowState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetFollowState_Request_

// alias to use template instance with default allocator
using SetFollowState_Request =
  turtlebot3_applications_msgs::srv::SetFollowState_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetFollowState_Request_<ContainerAllocator>::STOPPED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetFollowState_Request_<ContainerAllocator>::FOLLOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetFollowState_Request_<ContainerAllocator>::OK;
#endif  // __cplusplus < 201703L
// guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetFollowState_Request_<ContainerAllocator>::ERROR;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif

}  // namespace srv

}  // namespace turtlebot3_applications_msgs


#ifndef _WIN32
# define DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Response __declspec(deprecated)
#endif

namespace turtlebot3_applications_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetFollowState_Response_
{
  using Type = SetFollowState_Response_<ContainerAllocator>;

  explicit SetFollowState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  explicit SetFollowState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  // field types and members
  using _result_type =
    uint8_t;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Response
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__SetFollowState_Response
    std::shared_ptr<turtlebot3_applications_msgs::srv::SetFollowState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetFollowState_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetFollowState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetFollowState_Response_

// alias to use template instance with default allocator
using SetFollowState_Response =
  turtlebot3_applications_msgs::srv::SetFollowState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_applications_msgs

namespace turtlebot3_applications_msgs
{

namespace srv
{

struct SetFollowState
{
  using Request = turtlebot3_applications_msgs::srv::SetFollowState_Request;
  using Response = turtlebot3_applications_msgs::srv::SetFollowState_Response;
};

}  // namespace srv

}  // namespace turtlebot3_applications_msgs

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__STRUCT_HPP_
