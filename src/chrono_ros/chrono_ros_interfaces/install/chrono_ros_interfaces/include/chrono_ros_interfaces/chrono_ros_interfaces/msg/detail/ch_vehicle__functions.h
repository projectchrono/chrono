// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from chrono_ros_interfaces:msg/ChVehicle.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__FUNCTIONS_H_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "chrono_ros_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "chrono_ros_interfaces/msg/detail/ch_vehicle__struct.h"

/// Initialize msg/ChVehicle message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * chrono_ros_interfaces__msg__ChVehicle
 * )) before or use
 * chrono_ros_interfaces__msg__ChVehicle__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
bool
chrono_ros_interfaces__msg__ChVehicle__init(chrono_ros_interfaces__msg__ChVehicle * msg);

/// Finalize msg/ChVehicle message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
void
chrono_ros_interfaces__msg__ChVehicle__fini(chrono_ros_interfaces__msg__ChVehicle * msg);

/// Create msg/ChVehicle message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * chrono_ros_interfaces__msg__ChVehicle__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
chrono_ros_interfaces__msg__ChVehicle *
chrono_ros_interfaces__msg__ChVehicle__create();

/// Destroy msg/ChVehicle message.
/**
 * It calls
 * chrono_ros_interfaces__msg__ChVehicle__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
void
chrono_ros_interfaces__msg__ChVehicle__destroy(chrono_ros_interfaces__msg__ChVehicle * msg);

/// Check for msg/ChVehicle message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
bool
chrono_ros_interfaces__msg__ChVehicle__are_equal(const chrono_ros_interfaces__msg__ChVehicle * lhs, const chrono_ros_interfaces__msg__ChVehicle * rhs);

/// Copy a msg/ChVehicle message.
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
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
bool
chrono_ros_interfaces__msg__ChVehicle__copy(
  const chrono_ros_interfaces__msg__ChVehicle * input,
  chrono_ros_interfaces__msg__ChVehicle * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_type_hash_t *
chrono_ros_interfaces__msg__ChVehicle__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
chrono_ros_interfaces__msg__ChVehicle__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_runtime_c__type_description__TypeSource *
chrono_ros_interfaces__msg__ChVehicle__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
chrono_ros_interfaces__msg__ChVehicle__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/ChVehicle messages.
/**
 * It allocates the memory for the number of elements and calls
 * chrono_ros_interfaces__msg__ChVehicle__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
bool
chrono_ros_interfaces__msg__ChVehicle__Sequence__init(chrono_ros_interfaces__msg__ChVehicle__Sequence * array, size_t size);

/// Finalize array of msg/ChVehicle messages.
/**
 * It calls
 * chrono_ros_interfaces__msg__ChVehicle__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
void
chrono_ros_interfaces__msg__ChVehicle__Sequence__fini(chrono_ros_interfaces__msg__ChVehicle__Sequence * array);

/// Create array of msg/ChVehicle messages.
/**
 * It allocates the memory for the array and calls
 * chrono_ros_interfaces__msg__ChVehicle__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
chrono_ros_interfaces__msg__ChVehicle__Sequence *
chrono_ros_interfaces__msg__ChVehicle__Sequence__create(size_t size);

/// Destroy array of msg/ChVehicle messages.
/**
 * It calls
 * chrono_ros_interfaces__msg__ChVehicle__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
void
chrono_ros_interfaces__msg__ChVehicle__Sequence__destroy(chrono_ros_interfaces__msg__ChVehicle__Sequence * array);

/// Check for msg/ChVehicle message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
bool
chrono_ros_interfaces__msg__ChVehicle__Sequence__are_equal(const chrono_ros_interfaces__msg__ChVehicle__Sequence * lhs, const chrono_ros_interfaces__msg__ChVehicle__Sequence * rhs);

/// Copy an array of msg/ChVehicle messages.
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
ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
bool
chrono_ros_interfaces__msg__ChVehicle__Sequence__copy(
  const chrono_ros_interfaces__msg__ChVehicle__Sequence * input,
  chrono_ros_interfaces__msg__ChVehicle__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__FUNCTIONS_H_
