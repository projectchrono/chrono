#include "chrono_ros/ChROSInterface.h"

#include "chrono/core/ChTypes.h"
#include "chrono/core/ChLog.h"

namespace chrono {
namespace ros {

bool ChROSInterface::m_has_initialized = false;
rclcpp::Executor::SharedPtr ChROSInterface::m_executor = nullptr;

ChROSInterface::ChROSInterface(const std::string node_name) : m_node_name(node_name) {}

void ChROSInterface::Initialize() {
    if (!m_has_initialized) {
        GetLog() << "Initializing rclcpp. \n";
        rclcpp::init(0, 0);
        m_executor = chrono_types::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        m_has_initialized = true;
    }

    // TODO: Should we change the SignalHandlerOptions to None?
    m_node = std::make_shared<rclcpp::Node>(m_node_name);
    m_executor->add_node(m_node);

    GetLog() << "Initialized ChROSInterface: " << m_node_name.c_str() << ". \n";
}

void ChROSInterface::SpinSome() {
    if (rclcpp::ok()) {
        m_executor->spin_some();
    }
}

}  // namespace ros
}  // namespace chrono