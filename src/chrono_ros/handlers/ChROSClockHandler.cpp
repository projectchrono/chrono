#include "chrono_ros/handlers/ChROSClockHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

namespace chrono {
namespace ros {

ChROSClockHandler::ChROSClockHandler(uint64_t frequency) : ChROSHandler(frequency) {}

bool ChROSClockHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    rclcpp::ClockQoS qos;
    m_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);

    return true;
}

void ChROSClockHandler::Tick(double time) {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono