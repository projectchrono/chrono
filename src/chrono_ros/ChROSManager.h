// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Manager for the ROS handlers
//
// =============================================================================

#ifndef CH_ROS_MANAGER_H
#define CH_ROS_MANAGER_H

#include "chrono_ros/ChApiROS.h"

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>

namespace chrono {
namespace ros {

// Forward declarations
class ChROSInterface;
class ChROSHandler;
class ChROSIPCInterface;
namespace ipc {
    enum class MessageType : uint32_t;
}

/// @addtogroup ros_core
/// @{

/// Manages the ROS handlers and their registration/updates.
/// Uses IPC mode: handlers are not initialized in the main process.
/// Instead, their data is extracted and sent to a subprocess via IPC.
class CH_ROS_API ChROSManager {
  public:
    /// Constructor for the ChROSManager creates a single ChROSInterface. To use multiple ChROSInterfaces, multiple
    /// instances of this class should be used. Defaults node name is "chrono_ros_node".
    ChROSManager(const std::string& node_name = "chrono_ros_node");

    /// Initialize the ROS system. Prior to this, rclcpp::init() has not been called.
    void Initialize();

    /// Advance all handlers
    /// Returns false if rclcpp::ok() returns false
    bool Update(double time, double step);

    /// Register a new handler
    void RegisterHandler(std::shared_ptr<ChROSHandler> handler);

    /// Get the ChROSInterface
    std::shared_ptr<ChROSInterface> GetInterface() { return m_interface; }

  private:
    std::shared_ptr<ChROSInterface> m_interface;
    std::vector<std::shared_ptr<ChROSHandler>> m_handlers;
};

/// @}

}  // namespace ros
}  // namespace chrono

#endif
