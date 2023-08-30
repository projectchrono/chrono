// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from chrono_ros_interfaces:msg/ChVehicle.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "chrono_ros_interfaces/msg/detail/ch_vehicle__functions.h"
#include "chrono_ros_interfaces/msg/detail/ch_vehicle__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace chrono_ros_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ChVehicle_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) chrono_ros_interfaces::msg::ChVehicle(_init);
}

void ChVehicle_fini_function(void * message_memory)
{
  auto typed_message = static_cast<chrono_ros_interfaces::msg::ChVehicle *>(message_memory);
  typed_message->~ChVehicle();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ChVehicle_message_member_array[3] = {
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chrono_ros_interfaces::msg::ChVehicle, pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "twist",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Twist>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chrono_ros_interfaces::msg::ChVehicle, twist),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "accel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Accel>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chrono_ros_interfaces::msg::ChVehicle, accel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ChVehicle_message_members = {
  "chrono_ros_interfaces::msg",  // message namespace
  "ChVehicle",  // message name
  3,  // number of fields
  sizeof(chrono_ros_interfaces::msg::ChVehicle),
  ChVehicle_message_member_array,  // message members
  ChVehicle_init_function,  // function to initialize message memory (memory has to be allocated)
  ChVehicle_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ChVehicle_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ChVehicle_message_members,
  get_message_typesupport_handle_function,
  &chrono_ros_interfaces__msg__ChVehicle__get_type_hash,
  &chrono_ros_interfaces__msg__ChVehicle__get_type_description,
  &chrono_ros_interfaces__msg__ChVehicle__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace chrono_ros_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<chrono_ros_interfaces::msg::ChVehicle>()
{
  return &::chrono_ros_interfaces::msg::rosidl_typesupport_introspection_cpp::ChVehicle_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, chrono_ros_interfaces, msg, ChVehicle)() {
  return &::chrono_ros_interfaces::msg::rosidl_typesupport_introspection_cpp::ChVehicle_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
