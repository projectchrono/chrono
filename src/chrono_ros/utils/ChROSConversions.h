#ifndef CH_ROS_CONVERSIONS_H
#define CH_ROS_CONVERSIONS_H

#include "builtin_interfaces/msg/time.hpp"

namespace chrono {
namespace ros {

builtin_interfaces::msg::Time GetROSTimestamp(double elapsed_time_s);

}  // namespace ros
}  // namespace chrono

#endif