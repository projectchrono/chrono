// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chrono_ros_interfaces:srv/ChStartSimulation.idl
// generated code does not contain a copyright notice
#include "chrono_ros_interfaces/srv/detail/ch_start_simulation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
chrono_ros_interfaces__srv__ChStartSimulation_Request__init(chrono_ros_interfaces__srv__ChStartSimulation_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Request__fini(chrono_ros_interfaces__srv__ChStartSimulation_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Request__are_equal(const chrono_ros_interfaces__srv__ChStartSimulation_Request * lhs, const chrono_ros_interfaces__srv__ChStartSimulation_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Request__copy(
  const chrono_ros_interfaces__srv__ChStartSimulation_Request * input,
  chrono_ros_interfaces__srv__ChStartSimulation_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

chrono_ros_interfaces__srv__ChStartSimulation_Request *
chrono_ros_interfaces__srv__ChStartSimulation_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Request * msg = (chrono_ros_interfaces__srv__ChStartSimulation_Request *)allocator.allocate(sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Request));
  bool success = chrono_ros_interfaces__srv__ChStartSimulation_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Request__destroy(chrono_ros_interfaces__srv__ChStartSimulation_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chrono_ros_interfaces__srv__ChStartSimulation_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__init(chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Request * data = NULL;

  if (size) {
    data = (chrono_ros_interfaces__srv__ChStartSimulation_Request *)allocator.zero_allocate(size, sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chrono_ros_interfaces__srv__ChStartSimulation_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chrono_ros_interfaces__srv__ChStartSimulation_Request__fini(&data[i - 1]);
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
chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__fini(chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * array)
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
      chrono_ros_interfaces__srv__ChStartSimulation_Request__fini(&array->data[i]);
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

chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence *
chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * array = (chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence *)allocator.allocate(sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__destroy(chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__are_equal(const chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * lhs, const chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chrono_ros_interfaces__srv__ChStartSimulation_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__copy(
  const chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * input,
  chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chrono_ros_interfaces__srv__ChStartSimulation_Request * data =
      (chrono_ros_interfaces__srv__ChStartSimulation_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chrono_ros_interfaces__srv__ChStartSimulation_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chrono_ros_interfaces__srv__ChStartSimulation_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chrono_ros_interfaces__srv__ChStartSimulation_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
chrono_ros_interfaces__srv__ChStartSimulation_Response__init(chrono_ros_interfaces__srv__ChStartSimulation_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Response__fini(chrono_ros_interfaces__srv__ChStartSimulation_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Response__are_equal(const chrono_ros_interfaces__srv__ChStartSimulation_Response * lhs, const chrono_ros_interfaces__srv__ChStartSimulation_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Response__copy(
  const chrono_ros_interfaces__srv__ChStartSimulation_Response * input,
  chrono_ros_interfaces__srv__ChStartSimulation_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

chrono_ros_interfaces__srv__ChStartSimulation_Response *
chrono_ros_interfaces__srv__ChStartSimulation_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Response * msg = (chrono_ros_interfaces__srv__ChStartSimulation_Response *)allocator.allocate(sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Response));
  bool success = chrono_ros_interfaces__srv__ChStartSimulation_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Response__destroy(chrono_ros_interfaces__srv__ChStartSimulation_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chrono_ros_interfaces__srv__ChStartSimulation_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__init(chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Response * data = NULL;

  if (size) {
    data = (chrono_ros_interfaces__srv__ChStartSimulation_Response *)allocator.zero_allocate(size, sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chrono_ros_interfaces__srv__ChStartSimulation_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chrono_ros_interfaces__srv__ChStartSimulation_Response__fini(&data[i - 1]);
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
chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__fini(chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * array)
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
      chrono_ros_interfaces__srv__ChStartSimulation_Response__fini(&array->data[i]);
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

chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence *
chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * array = (chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence *)allocator.allocate(sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__destroy(chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__are_equal(const chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * lhs, const chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chrono_ros_interfaces__srv__ChStartSimulation_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__copy(
  const chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * input,
  chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chrono_ros_interfaces__srv__ChStartSimulation_Response * data =
      (chrono_ros_interfaces__srv__ChStartSimulation_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chrono_ros_interfaces__srv__ChStartSimulation_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chrono_ros_interfaces__srv__ChStartSimulation_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chrono_ros_interfaces__srv__ChStartSimulation_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "chrono_ros_interfaces/srv/detail/ch_start_simulation__functions.h"

bool
chrono_ros_interfaces__srv__ChStartSimulation_Event__init(chrono_ros_interfaces__srv__ChStartSimulation_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(msg);
    return false;
  }
  // request
  if (!chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__init(&msg->request, 0)) {
    chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(msg);
    return false;
  }
  // response
  if (!chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__init(&msg->response, 0)) {
    chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(msg);
    return false;
  }
  return true;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(chrono_ros_interfaces__srv__ChStartSimulation_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__fini(&msg->request);
  // response
  chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__fini(&msg->response);
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Event__are_equal(const chrono_ros_interfaces__srv__ChStartSimulation_Event * lhs, const chrono_ros_interfaces__srv__ChStartSimulation_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Event__copy(
  const chrono_ros_interfaces__srv__ChStartSimulation_Event * input,
  chrono_ros_interfaces__srv__ChStartSimulation_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!chrono_ros_interfaces__srv__ChStartSimulation_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!chrono_ros_interfaces__srv__ChStartSimulation_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

chrono_ros_interfaces__srv__ChStartSimulation_Event *
chrono_ros_interfaces__srv__ChStartSimulation_Event__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Event * msg = (chrono_ros_interfaces__srv__ChStartSimulation_Event *)allocator.allocate(sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Event));
  bool success = chrono_ros_interfaces__srv__ChStartSimulation_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Event__destroy(chrono_ros_interfaces__srv__ChStartSimulation_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__init(chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Event * data = NULL;

  if (size) {
    data = (chrono_ros_interfaces__srv__ChStartSimulation_Event *)allocator.zero_allocate(size, sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chrono_ros_interfaces__srv__ChStartSimulation_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(&data[i - 1]);
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
chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__fini(chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * array)
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
      chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(&array->data[i]);
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

chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence *
chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * array = (chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence *)allocator.allocate(sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__destroy(chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__are_equal(const chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * lhs, const chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chrono_ros_interfaces__srv__ChStartSimulation_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence__copy(
  const chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * input,
  chrono_ros_interfaces__srv__ChStartSimulation_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chrono_ros_interfaces__srv__ChStartSimulation_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chrono_ros_interfaces__srv__ChStartSimulation_Event * data =
      (chrono_ros_interfaces__srv__ChStartSimulation_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chrono_ros_interfaces__srv__ChStartSimulation_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chrono_ros_interfaces__srv__ChStartSimulation_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chrono_ros_interfaces__srv__ChStartSimulation_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
