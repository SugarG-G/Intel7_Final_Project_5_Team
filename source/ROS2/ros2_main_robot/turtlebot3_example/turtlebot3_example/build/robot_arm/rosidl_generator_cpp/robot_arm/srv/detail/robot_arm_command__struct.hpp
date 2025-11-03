// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_arm:srv/RobotArmCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__STRUCT_HPP_
#define ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_arm__srv__RobotArmCommand_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_arm__srv__RobotArmCommand_Request __declspec(deprecated)
#endif

namespace robot_arm
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RobotArmCommand_Request_
{
  using Type = RobotArmCommand_Request_<ContainerAllocator>;

  explicit RobotArmCommand_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
    }
  }

  explicit RobotArmCommand_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
    }
  }

  // field types and members
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;

  // setters for named parameter idiom
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_arm__srv__RobotArmCommand_Request
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_arm__srv__RobotArmCommand_Request
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotArmCommand_Request_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotArmCommand_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotArmCommand_Request_

// alias to use template instance with default allocator
using RobotArmCommand_Request =
  robot_arm::srv::RobotArmCommand_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_arm


#ifndef _WIN32
# define DEPRECATED__robot_arm__srv__RobotArmCommand_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_arm__srv__RobotArmCommand_Response __declspec(deprecated)
#endif

namespace robot_arm
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RobotArmCommand_Response_
{
  using Type = RobotArmCommand_Response_<ContainerAllocator>;

  explicit RobotArmCommand_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
    }
  }

  explicit RobotArmCommand_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
    }
  }

  // field types and members
  using _result_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_arm__srv__RobotArmCommand_Response
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_arm__srv__RobotArmCommand_Response
    std::shared_ptr<robot_arm::srv::RobotArmCommand_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotArmCommand_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotArmCommand_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotArmCommand_Response_

// alias to use template instance with default allocator
using RobotArmCommand_Response =
  robot_arm::srv::RobotArmCommand_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_arm

namespace robot_arm
{

namespace srv
{

struct RobotArmCommand
{
  using Request = robot_arm::srv::RobotArmCommand_Request;
  using Response = robot_arm::srv::RobotArmCommand_Response;
};

}  // namespace srv

}  // namespace robot_arm

#endif  // ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__STRUCT_HPP_
