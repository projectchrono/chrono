// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chrono_ros_interfaces:srv/ChStopSimulation.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_STOP_SIMULATION__STRUCT_H_
#define CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_STOP_SIMULATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ChStopSimulation in the package chrono_ros_interfaces.
typedef struct chrono_ros_interfaces__srv__ChStopSimulation_Request
{
  uint8_t structure_needs_at_least_one_member;
} chrono_ros_interfaces__srv__ChStopSimulation_Request;

// Struct for a sequence of chrono_ros_interfaces__srv__ChStopSimulation_Request.
typedef struct chrono_ros_interfaces__srv__ChStopSimulation_Request__Sequence
{
  chrono_ros_interfaces__srv__ChStopSimulation_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chrono_ros_interfaces__srv__ChStopSimulation_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/ChStopSimulation in the package chrono_ros_interfaces.
typedef struct chrono_ros_interfaces__srv__ChStopSimulation_Response
{
  bool success;
} chrono_ros_interfaces__srv__ChStopSimulation_Response;

// Struct for a sequence of chrono_ros_interfaces__srv__ChStopSimulation_Response.
typedef struct chrono_ros_interfaces__srv__ChStopSimulation_Response__Sequence
{
  chrono_ros_interfaces__srv__ChStopSimulation_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chrono_ros_interfaces__srv__ChStopSimulation_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  chrono_ros_interfaces__srv__ChStopSimulation_Event__request__MAX_SIZE = 1
};
// response
enum
{
  chrono_ros_interfaces__srv__ChStopSimulation_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ChStopSimulation in the package chrono_ros_interfaces.
typedef struct chrono_ros_interfaces__srv__ChStopSimulation_Event
{
  service_msgs__msg__ServiceEventInfo info;
  chrono_ros_interfaces__srv__ChStopSimulation_Request__Sequence request;
  chrono_ros_interfaces__srv__ChStopSimulation_Response__Sequence response;
} chrono_ros_interfaces__srv__ChStopSimulation_Event;

// Struct for a sequence of chrono_ros_interfaces__srv__ChStopSimulation_Event.
typedef struct chrono_ros_interfaces__srv__ChStopSimulation_Event__Sequence
{
  chrono_ros_interfaces__srv__ChStopSimulation_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chrono_ros_interfaces__srv__ChStopSimulation_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_STOP_SIMULATION__STRUCT_H_
