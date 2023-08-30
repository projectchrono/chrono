// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "chrono_ros_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "chrono_ros_interfaces/msg/detail/ch_driver_inputs__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace chrono_ros_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chrono_ros_interfaces
cdr_serialize(
  const chrono_ros_interfaces::msg::ChDriverInputs & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chrono_ros_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  chrono_ros_interfaces::msg::ChDriverInputs & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chrono_ros_interfaces
get_serialized_size(
  const chrono_ros_interfaces::msg::ChDriverInputs & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chrono_ros_interfaces
max_serialized_size_ChDriverInputs(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace chrono_ros_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chrono_ros_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, chrono_ros_interfaces, msg, ChDriverInputs)();

#ifdef __cplusplus
}
#endif

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
