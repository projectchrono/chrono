// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__BUILDER_HPP_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chrono_ros_interfaces/msg/detail/ch_driver_inputs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chrono_ros_interfaces
{

namespace msg
{

namespace builder
{

class Init_ChDriverInputs_braking
{
public:
  explicit Init_ChDriverInputs_braking(::chrono_ros_interfaces::msg::ChDriverInputs & msg)
  : msg_(msg)
  {}
  ::chrono_ros_interfaces::msg::ChDriverInputs braking(::chrono_ros_interfaces::msg::ChDriverInputs::_braking_type arg)
  {
    msg_.braking = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chrono_ros_interfaces::msg::ChDriverInputs msg_;
};

class Init_ChDriverInputs_throttle
{
public:
  explicit Init_ChDriverInputs_throttle(::chrono_ros_interfaces::msg::ChDriverInputs & msg)
  : msg_(msg)
  {}
  Init_ChDriverInputs_braking throttle(::chrono_ros_interfaces::msg::ChDriverInputs::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_ChDriverInputs_braking(msg_);
  }

private:
  ::chrono_ros_interfaces::msg::ChDriverInputs msg_;
};

class Init_ChDriverInputs_steering
{
public:
  explicit Init_ChDriverInputs_steering(::chrono_ros_interfaces::msg::ChDriverInputs & msg)
  : msg_(msg)
  {}
  Init_ChDriverInputs_throttle steering(::chrono_ros_interfaces::msg::ChDriverInputs::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_ChDriverInputs_throttle(msg_);
  }

private:
  ::chrono_ros_interfaces::msg::ChDriverInputs msg_;
};

class Init_ChDriverInputs_header
{
public:
  Init_ChDriverInputs_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChDriverInputs_steering header(::chrono_ros_interfaces::msg::ChDriverInputs::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChDriverInputs_steering(msg_);
  }

private:
  ::chrono_ros_interfaces::msg::ChDriverInputs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chrono_ros_interfaces::msg::ChDriverInputs>()
{
  return chrono_ros_interfaces::msg::builder::Init_ChDriverInputs_header();
}

}  // namespace chrono_ros_interfaces

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__BUILDER_HPP_
