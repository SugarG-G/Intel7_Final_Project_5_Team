// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_arm:srv/RobotArmCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__STRUCT_H_
#define ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RobotArmCommand in the package robot_arm.
typedef struct robot_arm__srv__RobotArmCommand_Request
{
  rosidl_runtime_c__String command;
} robot_arm__srv__RobotArmCommand_Request;

// Struct for a sequence of robot_arm__srv__RobotArmCommand_Request.
typedef struct robot_arm__srv__RobotArmCommand_Request__Sequence
{
  robot_arm__srv__RobotArmCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_arm__srv__RobotArmCommand_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RobotArmCommand in the package robot_arm.
typedef struct robot_arm__srv__RobotArmCommand_Response
{
  rosidl_runtime_c__String result;
} robot_arm__srv__RobotArmCommand_Response;

// Struct for a sequence of robot_arm__srv__RobotArmCommand_Response.
typedef struct robot_arm__srv__RobotArmCommand_Response__Sequence
{
  robot_arm__srv__RobotArmCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_arm__srv__RobotArmCommand_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_ARM__SRV__DETAIL__ROBOT_ARM_COMMAND__STRUCT_H_
