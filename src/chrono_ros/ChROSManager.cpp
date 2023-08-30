#include "chrono_ros/ChROSManager.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChTypes.h"

namespace chrono {
namespace ros {

ChROSManager::ChROSManager() : m_interface(chrono_types::make_shared<ChROSInterface>()) {
    m_interface->Initialize();
}

void ChROSManager::Advance(double step) {
    for (auto handler : m_registered_handlers)
        handler->Advance(step);

    m_interface->Advance();
}

void ChROSManager::RegisterHandler(std::shared_ptr<ChROSHandler> handler) {
    if (!handler->Initialize(m_interface)) {
        GetLog() << "Failed to initialize ROS handler with namespace " << handler->GetNamespace().c_str() << "\n";
        return;
    }
    m_registered_handlers.push_back(handler);
}

}  // namespace ros
}  // namespace chrono