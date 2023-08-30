// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chrono_ros_interfaces:msg/ChVehicle.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__STRUCT_H_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'accel'
#include "geometry_msgs/msg/detail/accel__struct.h"

/// Struct defined in msg/ChVehicle in the package chrono_ros_interfaces.
/**
  * Chrono ROS message with information coming from a ChVehicle
 */
typedef struct chrono_ros_interfaces__msg__ChVehicle
{
  /// Position/orientation, velocity, and acceleration information
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist twist;
  geometry_msgs__msg__Accel accel;
} chrono_ros_interfaces__msg__ChVehicle;

// Struct for a sequence of chrono_ros_interfaces__msg__ChVehicle.
typedef struct chrono_ros_interfaces__msg__ChVehicle__Sequence
{
  chrono_ros_interfaces__msg__ChVehicle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chrono_ros_interfaces__msg__ChVehicle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__STRUCT_H_
