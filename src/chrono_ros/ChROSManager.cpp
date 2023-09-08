#include "chrono_ros/ChROSManager.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChTypes.h"

namespace chrono {
namespace ros {

ChROSManager::ChROSManager() : m_interface(chrono_types::make_shared<ChROSInterface>()) {}

void ChROSManager::Initialize() {
    // Calls rclcpp::init()
    m_interface->Initialize();

    // Initialize the handlers. Print wanning and remove the handler if it fails to initialize.
    for (auto itr = m_handlers.begin(); itr != m_handlers.end();) {
        auto handler = *itr;
        if (!handler->Initialize(m_interface)) {
            GetLog() << "Failed to initialize ROS handler. Will remove handler and continue.\n";
            itr = m_handlers.erase(itr);
        } else {
            itr++;
        }
    }
}

bool ChROSManager::Update(double time, double step) {
    for (auto handler : m_handlers)
        handler->Update(time, step);

    m_interface->SpinSome();

    return rclcpp::ok();
}

void ChROSManager::RegisterHandler(std::shared_ptr<ChROSHandler> handler) {
    m_handlers.push_back(handler);
}

}  // namespace ros
}  // namespace chrono