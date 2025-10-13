// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_applications_msgs:msg/PanoramaImg.idl
// generated code does not contain a copyright notice
#include "turtlebot3_applications_msgs/msg/detail/panorama_img__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pano_id`
// Member `geo_tag`
#include "rosidl_runtime_c/string_functions.h"
// Member `image`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
turtlebot3_applications_msgs__msg__PanoramaImg__init(turtlebot3_applications_msgs__msg__PanoramaImg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    turtlebot3_applications_msgs__msg__PanoramaImg__fini(msg);
    return false;
  }
  // pano_id
  if (!rosidl_runtime_c__String__init(&msg->pano_id)) {
    turtlebot3_applications_msgs__msg__PanoramaImg__fini(msg);
    return false;
  }
  // latitude
  // longitude
  // heading
  // geo_tag
  if (!rosidl_runtime_c__String__init(&msg->geo_tag)) {
    turtlebot3_applications_msgs__msg__PanoramaImg__fini(msg);
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__init(&msg->image)) {
    turtlebot3_applications_msgs__msg__PanoramaImg__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_applications_msgs__msg__PanoramaImg__fini(turtlebot3_applications_msgs__msg__PanoramaImg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pano_id
  rosidl_runtime_c__String__fini(&msg->pano_id);
  // latitude
  // longitude
  // heading
  // geo_tag
  rosidl_runtime_c__String__fini(&msg->geo_tag);
  // image
  sensor_msgs__msg__Image__fini(&msg->image);
}

bool
turtlebot3_applications_msgs__msg__PanoramaImg__are_equal(const turtlebot3_applications_msgs__msg__PanoramaImg * lhs, const turtlebot3_applications_msgs__msg__PanoramaImg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // pano_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->pano_id), &(rhs->pano_id)))
  {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // geo_tag
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->geo_tag), &(rhs->geo_tag)))
  {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_applications_msgs__msg__PanoramaImg__copy(
  const turtlebot3_applications_msgs__msg__PanoramaImg * input,
  turtlebot3_applications_msgs__msg__PanoramaImg * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // pano_id
  if (!rosidl_runtime_c__String__copy(
      &(input->pano_id), &(output->pano_id)))
  {
    return false;
  }
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // heading
  output->heading = input->heading;
  // geo_tag
  if (!rosidl_runtime_c__String__copy(
      &(input->geo_tag), &(output->geo_tag)))
  {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  return true;
}

turtlebot3_applications_msgs__msg__PanoramaImg *
turtlebot3_applications_msgs__msg__PanoramaImg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__msg__PanoramaImg * msg = (turtlebot3_applications_msgs__msg__PanoramaImg *)allocator.allocate(sizeof(turtlebot3_applications_msgs__msg__PanoramaImg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_applications_msgs__msg__PanoramaImg));
  bool success = turtlebot3_applications_msgs__msg__PanoramaImg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot3_applications_msgs__msg__PanoramaImg__destroy(turtlebot3_applications_msgs__msg__PanoramaImg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot3_applications_msgs__msg__PanoramaImg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__init(turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__msg__PanoramaImg * data = NULL;

  if (size) {
    data = (turtlebot3_applications_msgs__msg__PanoramaImg *)allocator.zero_allocate(size, sizeof(turtlebot3_applications_msgs__msg__PanoramaImg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_applications_msgs__msg__PanoramaImg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_applications_msgs__msg__PanoramaImg__fini(&data[i - 1]);
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
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__fini(turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * array)
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
      turtlebot3_applications_msgs__msg__PanoramaImg__fini(&array->data[i]);
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

turtlebot3_applications_msgs__msg__PanoramaImg__Sequence *
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * array = (turtlebot3_applications_msgs__msg__PanoramaImg__Sequence *)allocator.allocate(sizeof(turtlebot3_applications_msgs__msg__PanoramaImg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__destroy(turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__are_equal(const turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * lhs, const turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_applications_msgs__msg__PanoramaImg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_applications_msgs__msg__PanoramaImg__Sequence__copy(
  const turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * input,
  turtlebot3_applications_msgs__msg__PanoramaImg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_applications_msgs__msg__PanoramaImg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot3_applications_msgs__msg__PanoramaImg * data =
      (turtlebot3_applications_msgs__msg__PanoramaImg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_applications_msgs__msg__PanoramaImg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot3_applications_msgs__msg__PanoramaImg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_applications_msgs__msg__PanoramaImg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
