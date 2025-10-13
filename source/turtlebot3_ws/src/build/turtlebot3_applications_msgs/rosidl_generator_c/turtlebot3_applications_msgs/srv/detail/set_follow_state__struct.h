// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_applications_msgs:srv/SetFollowState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__STRUCT_H_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STOPPED'.
enum
{
  turtlebot3_applications_msgs__srv__SetFollowState_Request__STOPPED = 0
};

/// Constant 'FOLLOW'.
enum
{
  turtlebot3_applications_msgs__srv__SetFollowState_Request__FOLLOW = 1
};

/// Constant 'OK'.
enum
{
  turtlebot3_applications_msgs__srv__SetFollowState_Request__OK = 0
};

/// Constant 'ERROR'.
enum
{
  turtlebot3_applications_msgs__srv__SetFollowState_Request__ERROR = 1
};

/// Struct defined in srv/SetFollowState in the package turtlebot3_applications_msgs.
typedef struct turtlebot3_applications_msgs__srv__SetFollowState_Request
{
  /// Messages
  ///
  /// STOPPED or FOLLOW
  uint8_t state;
} turtlebot3_applications_msgs__srv__SetFollowState_Request;

// Struct for a sequence of turtlebot3_applications_msgs__srv__SetFollowState_Request.
typedef struct turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence
{
  turtlebot3_applications_msgs__srv__SetFollowState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetFollowState in the package turtlebot3_applications_msgs.
typedef struct turtlebot3_applications_msgs__srv__SetFollowState_Response
{
  /// OK or ERROR
  uint8_t result;
} turtlebot3_applications_msgs__srv__SetFollowState_Response;

// Struct for a sequence of turtlebot3_applications_msgs__srv__SetFollowState_Response.
typedef struct turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence
{
  turtlebot3_applications_msgs__srv__SetFollowState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__SET_FOLLOW_STATE__STRUCT_H_
