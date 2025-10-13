// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_applications_msgs:srv/TakePanorama.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__STRUCT_H_
#define TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SNAPANDROTATE'.
/**
  * mode: rotate, stop, snapshot, rotate, stop, snapshot, ...
 */
enum
{
  turtlebot3_applications_msgs__srv__TakePanorama_Request__SNAPANDROTATE = 0
};

/// Constant 'CONTINUOUS'.
/**
  * mode: keep rotating while taking snapshots
 */
enum
{
  turtlebot3_applications_msgs__srv__TakePanorama_Request__CONTINUOUS = 1
};

/// Constant 'STOP'.
/**
  * mode: stop an ongoing panorama creation
 */
enum
{
  turtlebot3_applications_msgs__srv__TakePanorama_Request__STOP = 2
};

/// Constant 'STARTED'.
/**
  * status
 */
enum
{
  turtlebot3_applications_msgs__srv__TakePanorama_Request__STARTED = 0
};

/// Constant 'IN_PROGRESS'.
/**
  * status
 */
enum
{
  turtlebot3_applications_msgs__srv__TakePanorama_Request__IN_PROGRESS = 1
};

/// Constant 'STOPPED'.
/**
  * status
 */
enum
{
  turtlebot3_applications_msgs__srv__TakePanorama_Request__STOPPED = 2
};

/// Struct defined in srv/TakePanorama in the package turtlebot3_applications_msgs.
typedef struct turtlebot3_applications_msgs__srv__TakePanorama_Request
{
  /// Messages
  ///
  /// mode for taking the pictures
  uint8_t mode;
  /// total angle of panorama picture
  float pano_angle;
  /// angle interval when creating the panorama picture in snap&rotate mode, time interval otherwise
  float snap_interval;
  /// rotating velocity
  float rot_vel;
} turtlebot3_applications_msgs__srv__TakePanorama_Request;

// Struct for a sequence of turtlebot3_applications_msgs__srv__TakePanorama_Request.
typedef struct turtlebot3_applications_msgs__srv__TakePanorama_Request__Sequence
{
  turtlebot3_applications_msgs__srv__TakePanorama_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_applications_msgs__srv__TakePanorama_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/TakePanorama in the package turtlebot3_applications_msgs.
typedef struct turtlebot3_applications_msgs__srv__TakePanorama_Response
{
  uint8_t status;
} turtlebot3_applications_msgs__srv__TakePanorama_Response;

// Struct for a sequence of turtlebot3_applications_msgs__srv__TakePanorama_Response.
typedef struct turtlebot3_applications_msgs__srv__TakePanorama_Response__Sequence
{
  turtlebot3_applications_msgs__srv__TakePanorama_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_applications_msgs__srv__TakePanorama_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__SRV__DETAIL__TAKE_PANORAMA__STRUCT_H_
