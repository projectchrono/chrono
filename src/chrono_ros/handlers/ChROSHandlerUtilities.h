#ifndef CH_ROS_HANDLER_UTILITIES_H
#define CH_ROS_HANDLER_UTILITIES_H

#include "chrono_ros/ChROSInterface.h"

#include "builtin_interfaces/msg/time.hpp"

#include <string>

namespace chrono {
namespace ros {

class ChROSHandlerUtilities {
  public:
    static builtin_interfaces::msg::Time GetROSTimestamp(double elapsed_time_s);

    template <typename... Args>
    static std::string BuildRelativeTopicName(Args&&... args) {
        std::stringstream ss;
        ss << "~";
        ((ss << '/' << args), ...);
        return ss.str();
    }

    static bool CheckROSTopicName(std::shared_ptr<ChROSInterface> interface, const std::string& topic_name);
};

}  // namespace ros
}  // namespace chrono

#endif