// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chrono_ros_interfaces:srv/ChStartSimulation.idl
// generated code does not contain a copyright notice

#ifndef CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__TRAITS_HPP_
#define CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chrono_ros_interfaces/srv/detail/ch_start_simulation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace chrono_ros_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChStartSimulation_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChStartSimulation_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChStartSimulation_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace chrono_ros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chrono_ros_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chrono_ros_interfaces::srv::ChStartSimulation_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  chrono_ros_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chrono_ros_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chrono_ros_interfaces::srv::ChStartSimulation_Request & msg)
{
  return chrono_ros_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chrono_ros_interfaces::srv::ChStartSimulation_Request>()
{
  return "chrono_ros_interfaces::srv::ChStartSimulation_Request";
}

template<>
inline const char * name<chrono_ros_interfaces::srv::ChStartSimulation_Request>()
{
  return "chrono_ros_interfaces/srv/ChStartSimulation_Request";
}

template<>
struct has_fixed_size<chrono_ros_interfaces::srv::ChStartSimulation_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<chrono_ros_interfaces::srv::ChStartSimulation_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace chrono_ros_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChStartSimulation_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChStartSimulation_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChStartSimulation_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace chrono_ros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chrono_ros_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chrono_ros_interfaces::srv::ChStartSimulation_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  chrono_ros_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chrono_ros_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chrono_ros_interfaces::srv::ChStartSimulation_Response & msg)
{
  return chrono_ros_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chrono_ros_interfaces::srv::ChStartSimulation_Response>()
{
  return "chrono_ros_interfaces::srv::ChStartSimulation_Response";
}

template<>
inline const char * name<chrono_ros_interfaces::srv::ChStartSimulation_Response>()
{
  return "chrono_ros_interfaces/srv/ChStartSimulation_Response";
}

template<>
struct has_fixed_size<chrono_ros_interfaces::srv::ChStartSimulation_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<chrono_ros_interfaces::srv::ChStartSimulation_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace chrono_ros_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChStartSimulation_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChStartSimulation_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChStartSimulation_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace chrono_ros_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chrono_ros_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chrono_ros_interfaces::srv::ChStartSimulation_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  chrono_ros_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chrono_ros_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chrono_ros_interfaces::srv::ChStartSimulation_Event & msg)
{
  return chrono_ros_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chrono_ros_interfaces::srv::ChStartSimulation_Event>()
{
  return "chrono_ros_interfaces::srv::ChStartSimulation_Event";
}

template<>
inline const char * name<chrono_ros_interfaces::srv::ChStartSimulation_Event>()
{
  return "chrono_ros_interfaces/srv/ChStartSimulation_Event";
}

template<>
struct has_fixed_size<chrono_ros_interfaces::srv::ChStartSimulation_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation_Event>
  : std::integral_constant<bool, has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation_Request>::value && has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<chrono_ros_interfaces::srv::ChStartSimulation_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<chrono_ros_interfaces::srv::ChStartSimulation>()
{
  return "chrono_ros_interfaces::srv::ChStartSimulation";
}

template<>
inline const char * name<chrono_ros_interfaces::srv::ChStartSimulation>()
{
  return "chrono_ros_interfaces/srv/ChStartSimulation";
}

template<>
struct has_fixed_size<chrono_ros_interfaces::srv::ChStartSimulation>
  : std::integral_constant<
    bool,
    has_fixed_size<chrono_ros_interfaces::srv::ChStartSimulation_Request>::value &&
    has_fixed_size<chrono_ros_interfaces::srv::ChStartSimulation_Response>::value
  >
{
};

template<>
struct has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation>
  : std::integral_constant<
    bool,
    has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation_Request>::value &&
    has_bounded_size<chrono_ros_interfaces::srv::ChStartSimulation_Response>::value
  >
{
};

template<>
struct is_service<chrono_ros_interfaces::srv::ChStartSimulation>
  : std::true_type
{
};

template<>
struct is_service_request<chrono_ros_interfaces::srv::ChStartSimulation_Request>
  : std::true_type
{
};

template<>
struct is_service_response<chrono_ros_interfaces::srv::ChStartSimulation_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CHRONO_ROS_INTERFACES__SRV__DETAIL__CH_START_SIMULATION__TRAITS_HPP_
