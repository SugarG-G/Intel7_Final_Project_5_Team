// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_applications_msgs:srv/TakePanorama.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__STRUCT_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Request __declspec(deprecated)
#endif

namespace turtlebot3_applications_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TakePanorama_Request_
{
  using Type = TakePanorama_Request_<ContainerAllocator>;

  explicit TakePanorama_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->pano_angle = 0.0f;
      this->snap_interval = 0.0f;
      this->rot_vel = 0.0f;
    }
  }

  explicit TakePanorama_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->pano_angle = 0.0f;
      this->snap_interval = 0.0f;
      this->rot_vel = 0.0f;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _pano_angle_type =
    float;
  _pano_angle_type pano_angle;
  using _snap_interval_type =
    float;
  _snap_interval_type snap_interval;
  using _rot_vel_type =
    float;
  _rot_vel_type rot_vel;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__pano_angle(
    const float & _arg)
  {
    this->pano_angle = _arg;
    return *this;
  }
  Type & set__snap_interval(
    const float & _arg)
  {
    this->snap_interval = _arg;
    return *this;
  }
  Type & set__rot_vel(
    const float & _arg)
  {
    this->rot_vel = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SNAPANDROTATE =
    0u;
  static constexpr uint8_t CONTINUOUS =
    1u;
  static constexpr uint8_t STOP =
    2u;
  static constexpr uint8_t STARTED =
    0u;
  static constexpr uint8_t IN_PROGRESS =
    1u;
  static constexpr uint8_t STOPPED =
    2u;

  // pointer types
  using RawPtr =
    turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Request
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Request
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TakePanorama_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->pano_angle != other.pano_angle) {
      return false;
    }
    if (this->snap_interval != other.snap_interval) {
      return false;
    }
    if (this->rot_vel != other.rot_vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const TakePanorama_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TakePanorama_Request_

// alias to use template instance with default allocator
using TakePanorama_Request =
  turtlebot3_applications_msgs::srv::TakePanorama_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakePanorama_Request_<ContainerAllocator>::SNAPANDROTATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakePanorama_Request_<ContainerAllocator>::CONTINUOUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakePanorama_Request_<ContainerAllocator>::STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakePanorama_Request_<ContainerAllocator>::STARTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakePanorama_Request_<ContainerAllocator>::IN_PROGRESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TakePanorama_Request_<ContainerAllocator>::STOPPED;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace turtlebot3_applications_msgs


#ifndef _WIN32
# define DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Response __declspec(deprecated)
#endif

namespace turtlebot3_applications_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TakePanorama_Response_
{
  using Type = TakePanorama_Response_<ContainerAllocator>;

  explicit TakePanorama_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit TakePanorama_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Response
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_applications_msgs__srv__TakePanorama_Response
    std::shared_ptr<turtlebot3_applications_msgs::srv::TakePanorama_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TakePanorama_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const TakePanorama_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TakePanorama_Response_

// alias to use template instance with default allocator
using TakePanorama_Response =
  turtlebot3_applications_msgs::srv::TakePanorama_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_applications_msgs

namespace turtlebot3_applications_msgs
{

namespace srv
{

struct TakePanorama
{
  using Request = turtlebot3_applications_msgs::srv::TakePanorama_Request;
  using Response = turtlebot3_applications_msgs::srv::TakePanorama_Response;
};

}  // namespace srv

}  // namespace turtlebot3_applications_msgs

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__STRUCT_HPP_
