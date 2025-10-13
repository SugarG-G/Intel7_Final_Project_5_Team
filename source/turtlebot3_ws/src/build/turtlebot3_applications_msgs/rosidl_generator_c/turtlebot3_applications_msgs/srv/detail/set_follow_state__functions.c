// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_applications_msgs:srv/SetFollowState.idl
// generated code does not contain a copyright notice
#include "turtlebot3_applications_msgs/srv/detail/set_follow_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
turtlebot3_applications_msgs__srv__SetFollowState_Request__init(turtlebot3_applications_msgs__srv__SetFollowState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // state
  return true;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Request__fini(turtlebot3_applications_msgs__srv__SetFollowState_Request * msg)
{
  if (!msg) {
    return;
  }
  // state
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Request__are_equal(const turtlebot3_applications_msgs__srv__SetFollowState_Request * lhs, const turtlebot3_applications_msgs__srv__SetFollowState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Request__copy(
  const turtlebot3_applications_msgs__srv__SetFollowState_Request * input,
  turtlebot3_applications_msgs__srv__SetFollowState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  output->state = input->state;
  return true;
}

turtlebot3_applications_msgs__srv__SetFollowState_Request *
turtlebot3_applications_msgs__srv__SetFollowState_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__srv__SetFollowState_Request * msg = (turtlebot3_applications_msgs__srv__SetFollowState_Request *)allocator.allocate(sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Request));
  bool success = turtlebot3_applications_msgs__srv__SetFollowState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Request__destroy(turtlebot3_applications_msgs__srv__SetFollowState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot3_applications_msgs__srv__SetFollowState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__init(turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__srv__SetFollowState_Request * data = NULL;

  if (size) {
    data = (turtlebot3_applications_msgs__srv__SetFollowState_Request *)allocator.zero_allocate(size, sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_applications_msgs__srv__SetFollowState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_applications_msgs__srv__SetFollowState_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__fini(turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_applications_msgs__srv__SetFollowState_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence *
turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * array = (turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence *)allocator.allocate(sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__destroy(turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__are_equal(const turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * lhs, const turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_applications_msgs__srv__SetFollowState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence__copy(
  const turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * input,
  turtlebot3_applications_msgs__srv__SetFollowState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot3_applications_msgs__srv__SetFollowState_Request * data =
      (turtlebot3_applications_msgs__srv__SetFollowState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_applications_msgs__srv__SetFollowState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot3_applications_msgs__srv__SetFollowState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_applications_msgs__srv__SetFollowState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
turtlebot3_applications_msgs__srv__SetFollowState_Response__init(turtlebot3_applications_msgs__srv__SetFollowState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Response__fini(turtlebot3_applications_msgs__srv__SetFollowState_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Response__are_equal(const turtlebot3_applications_msgs__srv__SetFollowState_Response * lhs, const turtlebot3_applications_msgs__srv__SetFollowState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Response__copy(
  const turtlebot3_applications_msgs__srv__SetFollowState_Response * input,
  turtlebot3_applications_msgs__srv__SetFollowState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  return true;
}

turtlebot3_applications_msgs__srv__SetFollowState_Response *
turtlebot3_applications_msgs__srv__SetFollowState_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__srv__SetFollowState_Response * msg = (turtlebot3_applications_msgs__srv__SetFollowState_Response *)allocator.allocate(sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Response));
  bool success = turtlebot3_applications_msgs__srv__SetFollowState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Response__destroy(turtlebot3_applications_msgs__srv__SetFollowState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot3_applications_msgs__srv__SetFollowState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__init(turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__srv__SetFollowState_Response * data = NULL;

  if (size) {
    data = (turtlebot3_applications_msgs__srv__SetFollowState_Response *)allocator.zero_allocate(size, sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_applications_msgs__srv__SetFollowState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_applications_msgs__srv__SetFollowState_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__fini(turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_applications_msgs__srv__SetFollowState_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence *
turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * array = (turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence *)allocator.allocate(sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__destroy(turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__are_equal(const turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * lhs, const turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_applications_msgs__srv__SetFollowState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence__copy(
  const turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * input,
  turtlebot3_applications_msgs__srv__SetFollowState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_applications_msgs__srv__SetFollowState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot3_applications_msgs__srv__SetFollowState_Response * data =
      (turtlebot3_applications_msgs__srv__SetFollowState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_applications_msgs__srv__SetFollowState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot3_applications_msgs__srv__SetFollowState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_applications_msgs__srv__SetFollowState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
