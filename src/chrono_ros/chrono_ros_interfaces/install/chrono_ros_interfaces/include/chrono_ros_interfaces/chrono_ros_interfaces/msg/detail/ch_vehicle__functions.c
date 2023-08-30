// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chrono_ros_interfaces:msg/ChVehicle.idl
// generated code does not contain a copyright notice
#include "chrono_ros_interfaces/msg/detail/ch_vehicle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `accel`
#include "geometry_msgs/msg/detail/accel__functions.h"

bool
chrono_ros_interfaces__msg__ChVehicle__init(chrono_ros_interfaces__msg__ChVehicle * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    chrono_ros_interfaces__msg__ChVehicle__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    chrono_ros_interfaces__msg__ChVehicle__fini(msg);
    return false;
  }
  // accel
  if (!geometry_msgs__msg__Accel__init(&msg->accel)) {
    chrono_ros_interfaces__msg__ChVehicle__fini(msg);
    return false;
  }
  return true;
}

void
chrono_ros_interfaces__msg__ChVehicle__fini(chrono_ros_interfaces__msg__ChVehicle * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
  // accel
  geometry_msgs__msg__Accel__fini(&msg->accel);
}

bool
chrono_ros_interfaces__msg__ChVehicle__are_equal(const chrono_ros_interfaces__msg__ChVehicle * lhs, const chrono_ros_interfaces__msg__ChVehicle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  // accel
  if (!geometry_msgs__msg__Accel__are_equal(
      &(lhs->accel), &(rhs->accel)))
  {
    return false;
  }
  return true;
}

bool
chrono_ros_interfaces__msg__ChVehicle__copy(
  const chrono_ros_interfaces__msg__ChVehicle * input,
  chrono_ros_interfaces__msg__ChVehicle * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  // accel
  if (!geometry_msgs__msg__Accel__copy(
      &(input->accel), &(output->accel)))
  {
    return false;
  }
  return true;
}

chrono_ros_interfaces__msg__ChVehicle *
chrono_ros_interfaces__msg__ChVehicle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__msg__ChVehicle * msg = (chrono_ros_interfaces__msg__ChVehicle *)allocator.allocate(sizeof(chrono_ros_interfaces__msg__ChVehicle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chrono_ros_interfaces__msg__ChVehicle));
  bool success = chrono_ros_interfaces__msg__ChVehicle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chrono_ros_interfaces__msg__ChVehicle__destroy(chrono_ros_interfaces__msg__ChVehicle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chrono_ros_interfaces__msg__ChVehicle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chrono_ros_interfaces__msg__ChVehicle__Sequence__init(chrono_ros_interfaces__msg__ChVehicle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__msg__ChVehicle * data = NULL;

  if (size) {
    data = (chrono_ros_interfaces__msg__ChVehicle *)allocator.zero_allocate(size, sizeof(chrono_ros_interfaces__msg__ChVehicle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chrono_ros_interfaces__msg__ChVehicle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chrono_ros_interfaces__msg__ChVehicle__fini(&data[i - 1]);
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
chrono_ros_interfaces__msg__ChVehicle__Sequence__fini(chrono_ros_interfaces__msg__ChVehicle__Sequence * array)
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
      chrono_ros_interfaces__msg__ChVehicle__fini(&array->data[i]);
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

chrono_ros_interfaces__msg__ChVehicle__Sequence *
chrono_ros_interfaces__msg__ChVehicle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__msg__ChVehicle__Sequence * array = (chrono_ros_interfaces__msg__ChVehicle__Sequence *)allocator.allocate(sizeof(chrono_ros_interfaces__msg__ChVehicle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chrono_ros_interfaces__msg__ChVehicle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chrono_ros_interfaces__msg__ChVehicle__Sequence__destroy(chrono_ros_interfaces__msg__ChVehicle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chrono_ros_interfaces__msg__ChVehicle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chrono_ros_interfaces__msg__ChVehicle__Sequence__are_equal(const chrono_ros_interfaces__msg__ChVehicle__Sequence * lhs, const chrono_ros_interfaces__msg__ChVehicle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chrono_ros_interfaces__msg__ChVehicle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chrono_ros_interfaces__msg__ChVehicle__Sequence__copy(
  const chrono_ros_interfaces__msg__ChVehicle__Sequence * input,
  chrono_ros_interfaces__msg__ChVehicle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chrono_ros_interfaces__msg__ChVehicle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chrono_ros_interfaces__msg__ChVehicle * data =
      (chrono_ros_interfaces__msg__ChVehicle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chrono_ros_interfaces__msg__ChVehicle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chrono_ros_interfaces__msg__ChVehicle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chrono_ros_interfaces__msg__ChVehicle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
