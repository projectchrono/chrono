#include "chrono_ros/handlers/ChROSClockHandler.h"

#include "chrono_ros/ChROSInterface.h"

namespace chrono {
namespace ros {

ChROSClockHandler::ChROSClockHandler(ChSystem* system) : ChROSHandler(0), m_system(system) {}

bool ChROSClockHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    rclcpp::ClockQoS qos;
    m_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);

    return true;
}

void ChROSClockHandler::Tick() {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = GetTimestamp();
    m_publisher->publish(msg);
}

builtin_interfaces::msg::Time ChROSClockHandler::GetTimestamp() {
    const auto elapsed_time_s = m_system->GetChTime();

    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = static_cast<int32_t>(elapsed_time_s);
    timestamp.nanosec = static_cast<uint32_t>((elapsed_time_s - timestamp.sec) * 1e9);
    return timestamp;
}

}  // namespace ros
}  // namespace chrono