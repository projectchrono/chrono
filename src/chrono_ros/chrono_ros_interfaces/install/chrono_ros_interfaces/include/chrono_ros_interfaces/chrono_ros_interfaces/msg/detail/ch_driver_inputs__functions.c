// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice
#include "chrono_ros_interfaces/msg/detail/ch_driver_inputs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
chrono_ros_interfaces__msg__ChDriverInputs__init(chrono_ros_interfaces__msg__ChDriverInputs * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chrono_ros_interfaces__msg__ChDriverInputs__fini(msg);
    return false;
  }
  // steering
  // throttle
  // braking
  return true;
}

void
chrono_ros_interfaces__msg__ChDriverInputs__fini(chrono_ros_interfaces__msg__ChDriverInputs * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // steering
  // throttle
  // braking
}

bool
chrono_ros_interfaces__msg__ChDriverInputs__are_equal(const chrono_ros_interfaces__msg__ChDriverInputs * lhs, const chrono_ros_interfaces__msg__ChDriverInputs * rhs)
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
  // steering
  if (lhs->steering != rhs->steering) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // braking
  if (lhs->braking != rhs->braking) {
    return false;
  }
  return true;
}

bool
chrono_ros_interfaces__msg__ChDriverInputs__copy(
  const chrono_ros_interfaces__msg__ChDriverInputs * input,
  chrono_ros_interfaces__msg__ChDriverInputs * output)
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
  // steering
  output->steering = input->steering;
  // throttle
  output->throttle = input->throttle;
  // braking
  output->braking = input->braking;
  return true;
}

chrono_ros_interfaces__msg__ChDriverInputs *
chrono_ros_interfaces__msg__ChDriverInputs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__msg__ChDriverInputs * msg = (chrono_ros_interfaces__msg__ChDriverInputs *)allocator.allocate(sizeof(chrono_ros_interfaces__msg__ChDriverInputs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chrono_ros_interfaces__msg__ChDriverInputs));
  bool success = chrono_ros_interfaces__msg__ChDriverInputs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chrono_ros_interfaces__msg__ChDriverInputs__destroy(chrono_ros_interfaces__msg__ChDriverInputs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chrono_ros_interfaces__msg__ChDriverInputs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chrono_ros_interfaces__msg__ChDriverInputs__Sequence__init(chrono_ros_interfaces__msg__ChDriverInputs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__msg__ChDriverInputs * data = NULL;

  if (size) {
    data = (chrono_ros_interfaces__msg__ChDriverInputs *)allocator.zero_allocate(size, sizeof(chrono_ros_interfaces__msg__ChDriverInputs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chrono_ros_interfaces__msg__ChDriverInputs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chrono_ros_interfaces__msg__ChDriverInputs__fini(&data[i - 1]);
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
chrono_ros_interfaces__msg__ChDriverInputs__Sequence__fini(chrono_ros_interfaces__msg__ChDriverInputs__Sequence * array)
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
      chrono_ros_interfaces__msg__ChDriverInputs__fini(&array->data[i]);
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

chrono_ros_interfaces__msg__ChDriverInputs__Sequence *
chrono_ros_interfaces__msg__ChDriverInputs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__msg__ChDriverInputs__Sequence * array = (chrono_ros_interfaces__msg__ChDriverInputs__Sequence *)allocator.allocate(sizeof(chrono_ros_interfaces__msg__ChDriverInputs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chrono_ros_interfaces__msg__ChDriverInputs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chrono_ros_interfaces__msg__ChDriverInputs__Sequence__destroy(chrono_ros_interfaces__msg__ChDriverInputs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chrono_ros_interfaces__msg__ChDriverInputs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chrono_ros_interfaces__msg__ChDriverInputs__Sequence__are_equal(const chrono_ros_interfaces__msg__ChDriverInputs__Sequence * lhs, const chrono_ros_interfaces__msg__ChDriverInputs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chrono_ros_interfaces__msg__ChDriverInputs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chrono_ros_interfaces__msg__ChDriverInputs__Sequence__copy(
  const chrono_ros_interfaces__msg__ChDriverInputs__Sequence * input,
  chrono_ros_interfaces__msg__ChDriverInputs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chrono_ros_interfaces__msg__ChDriverInputs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chrono_ros_interfaces__msg__ChDriverInputs * data =
      (chrono_ros_interfaces__msg__ChDriverInputs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chrono_ros_interfaces__msg__ChDriverInputs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chrono_ros_interfaces__msg__ChDriverInputs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chrono_ros_interfaces__msg__ChDriverInputs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
