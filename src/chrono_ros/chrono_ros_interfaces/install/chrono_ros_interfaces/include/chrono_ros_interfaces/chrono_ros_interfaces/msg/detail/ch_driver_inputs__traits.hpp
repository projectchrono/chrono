// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__TRAITS_HPP_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chrono_ros_interfaces/msg/detail/ch_driver_inputs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace chrono_ros_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChDriverInputs & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: steering
  {
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
    out << ", ";
  }

  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: braking
  {
    out << "braking: ";
    rosidl_generator_traits::value_to_yaml(msg.braking, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChDriverInputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: steering
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
    out << "\n";
  }

  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: braking
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "braking: ";
    rosidl_generator_traits::value_to_yaml(msg.braking, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChDriverInputs & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace chrono_ros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chrono_ros_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chrono_ros_interfaces::msg::ChDriverInputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  chrono_ros_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chrono_ros_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const chrono_ros_interfaces::msg::ChDriverInputs & msg)
{
  return chrono_ros_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<chrono_ros_interfaces::msg::ChDriverInputs>()
{
  return "chrono_ros_interfaces::msg::ChDriverInputs";
}

template<>
inline const char * name<chrono_ros_interfaces::msg::ChDriverInputs>()
{
  return "chrono_ros_interfaces/msg/ChDriverInputs";
}

template<>
struct has_fixed_size<chrono_ros_interfaces::msg::ChDriverInputs>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<chrono_ros_interfaces::msg::ChDriverInputs>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<chrono_ros_interfaces::msg::ChDriverInputs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__TRAITS_HPP_
