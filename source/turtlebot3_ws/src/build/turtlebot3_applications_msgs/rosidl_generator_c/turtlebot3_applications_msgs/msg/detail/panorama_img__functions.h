// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from turtlebot3_applications_msgs:msg/PanoramaImg.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__FUNCTIONS_H_
#define TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot3_applications_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "turtlebot3_applications_msgs/msg/detail/panorama_img__struct.h"

/// Initialize msg/PanoramaImg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot3_applications_msgs__msg__PanoramaImg
 * )) before or use
 * turtlebot3_applications_msgs__msg__PanoramaImg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
bool
turtlebot3_applications_msgs__msg__PanoramaImg__init(turtlebot3_applications_msgs__msg__PanoramaImg * msg);

/// Finalize msg/PanoramaImg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
void
turtlebot3_applications_msgs__msg__PanoramaImg__fini(turtlebot3_applications_msgs__msg__PanoramaImg * msg);

/// Create msg/PanoramaImg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot3_applications_msgs__msg__PanoramaImg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
turtlebot3_applications_msgs__msg__PanoramaImg *
turtlebot3_applications_msgs__msg__PanoramaImg__create();

/// Destroy msg/PanoramaImg message.
/**
 * It calls
 * turtlebot3_applications_msgs__msg__PanoramaImg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
void
turtlebot3_applications_msgs__msg__PanoramaImg__destroy(turtlebot3_applications_msgs__msg__PanoramaImg * msg);

/// Check for msg/PanoramaImg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
bool
turtlebot3_applications_msgs__msg__PanoramaImg__are_equal(const turtlebot3_applications_msgs__msg__PanoramaImg * lhs, const turtlebot3_applications_msgs__msg__PanoramaImg * rhs);

/// Copy a msg/PanoramaImg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
bool
turtlebot3_applications_msgs__msg__PanoramaImg__copy(
  const turtlebot3_applications_msgs__msg__PanoramaImg * input,
  turtlebot3_applications_msgs__msg__PanoramaImg * output);

/// Initialize array of msg/PanoramaImg messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot3_applications_msgs__msg__PanoramaImg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
bool
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__init(turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * array, size_t size);

/// Finalize array of msg/PanoramaImg messages.
/**
 * It calls
 * turtlebot3_applications_msgs__msg__PanoramaImg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
void
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__fini(turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * array);

/// Create array of msg/PanoramaImg messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence *
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__create(size_t size);

/// Destroy array of msg/PanoramaImg messages.
/**
 * It calls
 * turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
void
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__destroy(turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * array);

/// Check for msg/PanoramaImg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
bool
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__are_equal(const turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * lhs, const turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * rhs);

/// Copy an array of msg/PanoramaImg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_applications_msgs
bool
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__copy(
  const turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * input,
  turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_APPLICATIONS_MSGS__MSG__DETAIL__PANORAMA_IMG__FUNCTIONS_H_
