// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "chrono_ros_interfaces/msg/detail/ch_driver_inputs__functions.h"
#include "chrono_ros_interfaces/msg/detail/ch_driver_inputs__struct.hpp"
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

void ChDriverInputs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) chrono_ros_interfaces::msg::ChDriverInputs(_init);
}

void ChDriverInputs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<chrono_ros_interfaces::msg::ChDriverInputs *>(message_memory);
  typed_message->~ChDriverInputs();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ChDriverInputs_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chrono_ros_interfaces::msg::ChDriverInputs, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "steering",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chrono_ros_interfaces::msg::ChDriverInputs, steering),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "throttle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chrono_ros_interfaces::msg::ChDriverInputs, throttle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "braking",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chrono_ros_interfaces::msg::ChDriverInputs, braking),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ChDriverInputs_message_members = {
  "chrono_ros_interfaces::msg",  // message namespace
  "ChDriverInputs",  // message name
  4,  // number of fields
  sizeof(chrono_ros_interfaces::msg::ChDriverInputs),
  ChDriverInputs_message_member_array,  // message members
  ChDriverInputs_init_function,  // function to initialize message memory (memory has to be allocated)
  ChDriverInputs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ChDriverInputs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ChDriverInputs_message_members,
  get_message_typesupport_handle_function,
  &chrono_ros_interfaces__msg__ChDriverInputs__get_type_hash,
  &chrono_ros_interfaces__msg__ChDriverInputs__get_type_description,
  &chrono_ros_interfaces__msg__ChDriverInputs__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace chrono_ros_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<chrono_ros_interfaces::msg::ChDriverInputs>()
{
  return &::chrono_ros_interfaces::msg::rosidl_typesupport_introspection_cpp::ChDriverInputs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, chrono_ros_interfaces, msg, ChDriverInputs)() {
  return &::chrono_ros_interfaces::msg::rosidl_typesupport_introspection_cpp::ChDriverInputs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
