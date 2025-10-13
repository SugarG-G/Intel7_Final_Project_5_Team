// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_applications_msgs:msg/PanoramaImg.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__STRUCT_H_
#define TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pano_id'
// Member 'geo_tag'
#include "rosidl_runtime_c/string.h"
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/PanoramaImg in the package turtlebot3_applications_msgs.
/**
  * Messages
 */
typedef struct turtlebot3_applications_msgs__msg__PanoramaImg
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String pano_id;
  double latitude;
  double longitude;
  /// in degrees, compass heading
  double heading;
  rosidl_runtime_c__String geo_tag;
  sensor_msgs__msg__Image image;
} turtlebot3_applications_msgs__msg__PanoramaImg;

// Struct for a sequence of turtlebot3_applications_msgs__msg__PanoramaImg.
typedef struct turtlebot3_applications_msgs__msg__PanoramaImg__Sequence
{
  turtlebot3_applications_msgs__msg__PanoramaImg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_applications_msgs__msg__PanoramaImg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__STRUCT_H_
