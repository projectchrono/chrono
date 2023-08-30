#ifndef CH_ROS_MANAGER_H
#define CH_ROS_MANAGER_H

#include <vector>
#include <memory>

#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/ChROSHandler.h"

namespace chrono {
namespace ros {

/// Managers the ROS handlers and their registration/updates
class ChROSManager {
  public:
    ChROSManager();

    /// Advance all handlers
    void Advance(double step);

    /// Register a new handler
    void RegisterHandler(std::shared_ptr<ChROSHandler> handler);

  private:
    std::shared_ptr<ChROSInterface> m_interface;
    std::vector<std::shared_ptr<ChROSHandler>> m_registered_handlers;
};

}  // namespace ros
}  // namespace chrono

#endif
