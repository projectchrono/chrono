// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from chrono_ros_interfaces:srv/ChStartSimulation.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__TYPE_SUPPORT_H_
#define CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "chrono_ros_interfaces/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  chrono_ros_interfaces,
  srv,
  ChStartSimulation_Request
)();

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  chrono_ros_interfaces,
  srv,
  ChStartSimulation_Response
)();

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  chrono_ros_interfaces,
  srv,
  ChStartSimulation_Event
)();

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  chrono_ros_interfaces,
  srv,
  ChStartSimulation
)();

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  chrono_ros_interfaces,
  srv,
  ChStartSimulation
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  chrono_ros_interfaces,
  srv,
  ChStartSimulation
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__TYPE_SUPPORT_H_
