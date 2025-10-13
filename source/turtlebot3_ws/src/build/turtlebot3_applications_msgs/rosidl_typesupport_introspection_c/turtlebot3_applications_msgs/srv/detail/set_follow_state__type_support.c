// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from turtlebot3_applications_msgs:srv/SetFollowState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "turtlebot3_applications_msgs/srv/detail/set_follow_state__rosidl_typesupport_introspection_c.h"
#include "turtlebot3_applications_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "turtlebot3_applications_msgs/srv/detail/set_follow_state__functions.h"
#include "turtlebot3_applications_msgs/srv/detail/set_follow_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtlebot3_applications_msgs__srv__SetFollowState_Request__init(message_memory);
}

void turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_fini_function(void * message_memory)
{
  turtlebot3_applications_msgs__srv__SetFollowState_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_member_array[1] = {
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__srv__SetFollowState_Request, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_members = {
  "turtlebot3_applications_msgs__srv",  // message namespace
  "SetFollowState_Request",  // message name
  1,  // number of fields
  sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Request),
  turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_member_array,  // message members
  turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_type_support_handle = {
  0,
  &turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_applications_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, srv, SetFollowState_Request)() {
  if (!turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_type_support_handle.typesupport_identifier) {
    turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtlebot3_applications_msgs__srv__SetFollowState_Request__rosidl_typesupport_introspection_c__SetFollowState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "turtlebot3_applications_msgs/srv/detail/set_follow_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "turtlebot3_applications_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "turtlebot3_applications_msgs/srv/detail/set_follow_state__functions.h"
// already included above
// #include "turtlebot3_applications_msgs/srv/detail/set_follow_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtlebot3_applications_msgs__srv__SetFollowState_Response__init(message_memory);
}

void turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_fini_function(void * message_memory)
{
  turtlebot3_applications_msgs__srv__SetFollowState_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_member_array[1] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_applications_msgs__srv__SetFollowState_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_members = {
  "turtlebot3_applications_msgs__srv",  // message namespace
  "SetFollowState_Response",  // message name
  1,  // number of fields
  sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Response),
  turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_member_array,  // message members
  turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_type_support_handle = {
  0,
  &turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_applications_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, srv, SetFollowState_Response)() {
  if (!turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_type_support_handle.typesupport_identifier) {
    turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtlebot3_applications_msgs__srv__SetFollowState_Response__rosidl_typesupport_introspection_c__SetFollowState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "turtlebot3_applications_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "turtlebot3_applications_msgs/srv/detail/set_follow_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_service_members = {
  "turtlebot3_applications_msgs__srv",  // service namespace
  "SetFollowState",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_Request_message_type_support_handle,
  NULL  // response message
  // turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_Response_message_type_support_handle
};

static rosidl_service_type_support_t turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_service_type_support_handle = {
  0,
  &turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, srv, SetFollowState_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, srv, SetFollowState_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_applications_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, srv, SetFollowState)() {
  if (!turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_service_type_support_handle.typesupport_identifier) {
    turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, srv, SetFollowState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_applications_msgs, srv, SetFollowState_Response)()->data;
  }

  return &turtlebot3_applications_msgs__srv__detail__set_follow_state__rosidl_typesupport_introspection_c__SetFollowState_service_type_support_handle;
}
