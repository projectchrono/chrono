// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chrono_ros_interfaces:srv/ChStartSimulation.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__BUILDER_HPP_
#define CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chrono_ros_interfaces/srv/detail/ch_start_simulation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chrono_ros_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chrono_ros_interfaces::srv::ChStartSimulation_Request>()
{
  return ::chrono_ros_interfaces::srv::ChStartSimulation_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace chrono_ros_interfaces


namespace chrono_ros_interfaces
{

namespace srv
{

namespace builder
{

class Init_ChStartSimulation_Response_success
{
public:
  Init_ChStartSimulation_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::chrono_ros_interfaces::srv::ChStartSimulation_Response success(::chrono_ros_interfaces::srv::ChStartSimulation_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chrono_ros_interfaces::srv::ChStartSimulation_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chrono_ros_interfaces::srv::ChStartSimulation_Response>()
{
  return chrono_ros_interfaces::srv::builder::Init_ChStartSimulation_Response_success();
}

}  // namespace chrono_ros_interfaces


namespace chrono_ros_interfaces
{

namespace srv
{

namespace builder
{

class Init_ChStartSimulation_Event_response
{
public:
  explicit Init_ChStartSimulation_Event_response(::chrono_ros_interfaces::srv::ChStartSimulation_Event & msg)
  : msg_(msg)
  {}
  ::chrono_ros_interfaces::srv::ChStartSimulation_Event response(::chrono_ros_interfaces::srv::ChStartSimulation_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chrono_ros_interfaces::srv::ChStartSimulation_Event msg_;
};

class Init_ChStartSimulation_Event_request
{
public:
  explicit Init_ChStartSimulation_Event_request(::chrono_ros_interfaces::srv::ChStartSimulation_Event & msg)
  : msg_(msg)
  {}
  Init_ChStartSimulation_Event_response request(::chrono_ros_interfaces::srv::ChStartSimulation_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ChStartSimulation_Event_response(msg_);
  }

private:
  ::chrono_ros_interfaces::srv::ChStartSimulation_Event msg_;
};

class Init_ChStartSimulation_Event_info
{
public:
  Init_ChStartSimulation_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChStartSimulation_Event_request info(::chrono_ros_interfaces::srv::ChStartSimulation_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ChStartSimulation_Event_request(msg_);
  }

private:
  ::chrono_ros_interfaces::srv::ChStartSimulation_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chrono_ros_interfaces::srv::ChStartSimulation_Event>()
{
  return chrono_ros_interfaces::srv::builder::Init_ChStartSimulation_Event_info();
}

}  // namespace chrono_ros_interfaces

#endif  // CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__BUILDER_HPP_
