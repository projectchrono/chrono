// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chrono_ros_interfaces:msg/ChVehicle.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__STRUCT_HPP_
#define CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'accel'
#include "geometry_msgs/msg/detail/accel__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chrono_ros_interfaces__msg__ChVehicle __attribute__((deprecated))
#else
# define DEPRECATED__chrono_ros_interfaces__msg__ChVehicle __declspec(deprecated)
#endif

namespace chrono_ros_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChVehicle_
{
  using Type = ChVehicle_<ContainerAllocator>;

  explicit ChVehicle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    twist(_init),
    accel(_init)
  {
    (void)_init;
  }

  explicit ChVehicle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    twist(_alloc, _init),
    accel(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _accel_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _accel_type accel;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__accel(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->accel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator> *;
  using ConstRawPtr =
    const chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chrono_ros_interfaces__msg__ChVehicle
    std::shared_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chrono_ros_interfaces__msg__ChVehicle
    std::shared_ptr<chrono_ros_interfaces::msg::ChVehicle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChVehicle_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->accel != other.accel) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChVehicle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChVehicle_

// alias to use template instance with default allocator
using ChVehicle =
  chrono_ros_interfaces::msg::ChVehicle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chrono_ros_interfaces

#endif  // CHRONO_ROS_INTERFACES__MSG__DETAIL__CH_VEHICLE__STRUCT_HPP_
