// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__STRUCT_H_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ChDriverInputs in the package chrono_ros_interfaces.
/**
  * Input message that is to be sent to the vehicle
  *
  * The input includes steering, throttle, and braking. 
 */
typedef struct chrono_ros_interfaces__msg__ChDriverInputs
{
  std_msgs__msg__Header header;
  /// Steering value [-1,1]
  /// -1 indicates turn fully to the left, 1 indicates turn fully to the right
  /// Independent of the max turn of the vehicle hardware
  double steering;
  /// Throttle value [0, 1]
  /// 0 indicates no throttle, 1 indicates 100% throttle
  /// Independent of the acceleration profile for the car
  double throttle;
  /// Braking value [0, 1]
  /// 0 indicates no braking, 1 indicates 100% braking
  double braking;
} chrono_ros_interfaces__msg__ChDriverInputs;

// Struct for a sequence of chrono_ros_interfaces__msg__ChDriverInputs.
typedef struct chrono_ros_interfaces__msg__ChDriverInputs__Sequence
{
  chrono_ros_interfaces__msg__ChDriverInputs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chrono_ros_interfaces__msg__ChDriverInputs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__STRUCT_H_
