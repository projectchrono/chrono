#include "chrono_ros/ChROSInterface.h"

#include "chrono/core/ChTypes.h"
#include "chrono/core/ChLog.h"

namespace chrono {
namespace ros {

ChROSInterface::ChROSInterface(const std::string node_name) : m_node_name(node_name) {}

void ChROSInterface::Initialize() {
    rclcpp::init(0, 0);
    m_node = std::make_shared<rclcpp::Node>(m_node_name);
    m_executor = chrono_types::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(m_node);

    GetLog() << "Initialized ChROSInterface. \n";
}

void ChROSInterface::Advance() {
    if (rclcpp::ok()) {
        m_executor->spin_some();
    }
}

}  // namespace ros
}  // namespace chrono