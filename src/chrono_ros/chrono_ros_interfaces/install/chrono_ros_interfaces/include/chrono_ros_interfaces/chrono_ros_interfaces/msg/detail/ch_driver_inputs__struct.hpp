// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__STRUCT_HPP_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chrono_ros_interfaces__msg__ChDriverInputs __attribute__((deprecated))
#else
# define DEPRECATED__chrono_ros_interfaces__msg__ChDriverInputs __declspec(deprecated)
#endif

namespace chrono_ros_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChDriverInputs_
{
  using Type = ChDriverInputs_<ContainerAllocator>;

  explicit ChDriverInputs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering = 0.0;
      this->throttle = 0.0;
      this->braking = 0.0;
    }
  }

  explicit ChDriverInputs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering = 0.0;
      this->throttle = 0.0;
      this->braking = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _steering_type =
    double;
  _steering_type steering;
  using _throttle_type =
    double;
  _throttle_type throttle;
  using _braking_type =
    double;
  _braking_type braking;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__steering(
    const double & _arg)
  {
    this->steering = _arg;
    return *this;
  }
  Type & set__throttle(
    const double & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__braking(
    const double & _arg)
  {
    this->braking = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator> *;
  using ConstRawPtr =
    const chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chrono_ros_interfaces__msg__ChDriverInputs
    std::shared_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chrono_ros_interfaces__msg__ChDriverInputs
    std::shared_ptr<chrono_ros_interfaces::msg::ChDriverInputs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChDriverInputs_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->steering != other.steering) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->braking != other.braking) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChDriverInputs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChDriverInputs_

// alias to use template instance with default allocator
using ChDriverInputs =
  chrono_ros_interfaces::msg::ChDriverInputs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chrono_ros_interfaces

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_DRIVER_INPUTS__STRUCT_HPP_
