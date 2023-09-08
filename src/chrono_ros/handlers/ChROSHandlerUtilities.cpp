#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "chrono/core/ChLog.h"

namespace chrono {
namespace ros {

builtin_interfaces::msg::Time ChROSHandlerUtilities::GetROSTimestamp(double elapsed_time_s) {
    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = static_cast<int32_t>(elapsed_time_s);
    timestamp.nanosec = static_cast<uint32_t>((elapsed_time_s - timestamp.sec) * 1e9);
    return timestamp;
}

bool ChROSHandlerUtilities::CheckROSTopicName(std::shared_ptr<ChROSInterface> interface,
                                              const std::string& topic_name) {
    try {
        std::string full_topic_name = rclcpp::expand_topic_or_service_name(topic_name, interface->GetNode()->get_name(),
                                                                           interface->GetNode()->get_namespace());
    } catch (rclcpp::exceptions::InvalidTopicNameError& e) {
        GetLog() << "ERROR: Topic '" << topic_name.c_str() << "' is not a valid ROS topic name: " << e.what() << "\n";
        return false;
    }

    return true;
}

}  // namespace ros
}  // namespace chrono