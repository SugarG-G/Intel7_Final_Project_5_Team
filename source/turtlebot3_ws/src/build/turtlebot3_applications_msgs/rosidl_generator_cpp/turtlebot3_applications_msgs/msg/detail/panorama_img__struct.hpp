// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_applications_msgs:msg/PanoramaImg.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__STRUCT_HPP_
#define TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_applications_msgs__msg__PanoramaImg __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_applications_msgs__msg__PanoramaImg __declspec(deprecated)
#endif

namespace turtlebot3_applications_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PanoramaImg_
{
  using Type = PanoramaImg_<ContainerAllocator>;

  explicit PanoramaImg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    image(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pano_id = "";
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->heading = 0.0;
      this->geo_tag = "";
    }
  }

  explicit PanoramaImg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pano_id(_alloc),
    geo_tag(_alloc),
    image(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pano_id = "";
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->heading = 0.0;
      this->geo_tag = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pano_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pano_id_type pano_id;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _heading_type =
    double;
  _heading_type heading;
  using _geo_tag_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _geo_tag_type geo_tag;
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pano_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pano_id = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__heading(
    const double & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__geo_tag(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->geo_tag = _arg;
    return *this;
  }
  Type & set__image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_applications_msgs__msg__PanoramaImg
    std::shared_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_applications_msgs__msg__PanoramaImg
    std::shared_ptr<turtlebot3_applications_msgs::msg::PanoramaImg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PanoramaImg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pano_id != other.pano_id) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->geo_tag != other.geo_tag) {
      return false;
    }
    if (this->image != other.image) {
      return false;
    }
    return true;
  }
  bool operator!=(const PanoramaImg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PanoramaImg_

// alias to use template instance with default allocator
using PanoramaImg =
  turtlebot3_applications_msgs::msg::PanoramaImg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turtlebot3_applications_msgs

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__STRUCT_HPP_
