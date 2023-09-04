#include "chrono_ros/utils/ChROSConversions.h"

namespace chrono {
namespace ros {

builtin_interfaces::msg::Time GetROSTimestamp(double elapsed_time_s) {
    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = static_cast<int32_t>(elapsed_time_s);
    timestamp.nanosec = static_cast<uint32_t>((elapsed_time_s - timestamp.sec) * 1e9);
    return timestamp;
}

}  // namespace ros
}  // namespace chrono