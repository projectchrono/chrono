#include "chrono_ros/handlers/ChROSClockHandler.h"

#include "chrono_ros/utils/ChROSConversions.h"

namespace chrono {
namespace ros {

ChROSClockHandler::ChROSClockHandler() : ChROSHandler() {}

bool ChROSClockHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    rclcpp::ClockQoS qos;
    m_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(ResolveROSName("clock"), qos);

    return true;
}

void ChROSClockHandler::Tick(double time) {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = GetROSTimestamp(time);
    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono