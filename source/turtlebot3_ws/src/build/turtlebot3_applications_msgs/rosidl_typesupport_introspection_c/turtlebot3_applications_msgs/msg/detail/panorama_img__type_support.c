// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from turtlebot3_applications_msgs:msg/PanoramaImg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "turtlebot3_applications_msgs/msg/detail/panorama_img__rosidl_typesupport_introspection_c.h"
#include "turtlebot3_applications_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "turtlebot3_applications_msgs/msg/detail/panorama_img__functions.h"
#include "turtlebot3_applications_msgs/msg/detail/panorama_img__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pano_id`
// Member `geo_tag`
#include "rosidl_runtime_c/string_functions.h"
// Member `image`
#include "sensor_msgs/msg/image.h"
// Member `image`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtlebot3_applications_msgs__msg__PanoramaImg__init(message_memory);
}

void turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_fini_function(void * message_memory)
{
  turtlebot3_applications_msgs__msg__PanoramaImg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__msg__PanoramaImg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pano_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__msg__PanoramaImg, pano_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__msg__PanoramaImg, latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__msg__PanoramaImg, longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "heading",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__msg__PanoramaImg, heading),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "geo_tag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__msg__PanoramaImg, geo_tag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__msg__PanoramaImg, image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_members = {
  "turtlebot3_applications_msgs__msg",  // message namespace
  "PanoramaImg",  // message name
  7,  // number of fields
  sizeof(turtlebot3_applications_msgs__msg__PanoramaImg),
  turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_member_array,  // message members
  turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_init_function,  // function to initialize message memory (memory has to be allocated)
  turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_type_support_handle = {
  0,
  &turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_applications_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, msg, PanoramaImg)() {
  turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_type_support_handle.typesupport_identifier) {
    turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtlebot3_applications_msgs__msg__PanoramaImg__rosidl_typesupport_introspection_c__PanoramaImg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
