// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chrono_ros_interfaces:msg/ChVehicle.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__BUILDER_HPP_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chrono_ros_interfaces/msg/detail/ch_vehicle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chrono_ros_interfaces
{

namespace msg
{

namespace builder
{

class Init_ChVehicle_accel
{
public:
  explicit Init_ChVehicle_accel(::chrono_ros_interfaces::msg::ChVehicle & msg)
  : msg_(msg)
  {}
  ::chrono_ros_interfaces::msg::ChVehicle accel(::chrono_ros_interfaces::msg::ChVehicle::_accel_type arg)
  {
    msg_.accel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chrono_ros_interfaces::msg::ChVehicle msg_;
};

class Init_ChVehicle_twist
{
public:
  explicit Init_ChVehicle_twist(::chrono_ros_interfaces::msg::ChVehicle & msg)
  : msg_(msg)
  {}
  Init_ChVehicle_accel twist(::chrono_ros_interfaces::msg::ChVehicle::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_ChVehicle_accel(msg_);
  }

private:
  ::chrono_ros_interfaces::msg::ChVehicle msg_;
};

class Init_ChVehicle_pose
{
public:
  Init_ChVehicle_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChVehicle_twist pose(::chrono_ros_interfaces::msg::ChVehicle::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ChVehicle_twist(msg_);
  }

private:
  ::chrono_ros_interfaces::msg::ChVehicle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chrono_ros_interfaces::msg::ChVehicle>()
{
  return chrono_ros_interfaces::msg::builder::Init_ChVehicle_pose();
}

}  // namespace chrono_ros_interfaces

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__BUILDER_HPP_
