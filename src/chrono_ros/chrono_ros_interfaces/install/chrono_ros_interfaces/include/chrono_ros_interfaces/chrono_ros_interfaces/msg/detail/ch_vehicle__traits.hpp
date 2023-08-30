// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chrono_ros_interfaces:msg/ChVehicle.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__TRAITS_HPP_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chrono_ros_interfaces/msg/detail/ch_vehicle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'accel'
#include "geometry_msgs/msg/detail/accel__traits.hpp"

namespace chrono_ros_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChVehicle & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: accel
  {
    out << "accel: ";
    to_flow_style_yaml(msg.accel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChVehicle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel:\n";
    to_block_style_yaml(msg.accel, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChVehicle & msg, bool use_flow_style = false)
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
  const chrono_ros_interfaces::msg::ChVehicle & msg,
  std::ostream & out, size_t indentation = 0)
{
  chrono_ros_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chrono_ros_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const chrono_ros_interfaces::msg::ChVehicle & msg)
{
  return chrono_ros_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<chrono_ros_interfaces::msg::ChVehicle>()
{
  return "chrono_ros_interfaces::msg::ChVehicle";
}

template<>
inline const char * name<chrono_ros_interfaces::msg::ChVehicle>()
{
  return "chrono_ros_interfaces/msg/ChVehicle";
}

template<>
struct has_fixed_size<chrono_ros_interfaces::msg::ChVehicle>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Accel>::value && has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<chrono_ros_interfaces::msg::ChVehicle>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Accel>::value && has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<chrono_ros_interfaces::msg::ChVehicle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__TRAITS_HPP_
