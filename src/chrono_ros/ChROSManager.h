// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
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

namespace chrono {
namespace ros {

// Forward declarations
class ChROSInterface;
class ChROSHandler;
#ifdef CHRONO_VSG
class ChROSIPCInterface;
#endif

/// @addtogroup ros_core
/// @{

/// Managers the ROS handlers and their registration/updates
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
    
#ifdef CHRONO_VSG
    /// Serialize handler data for IPC transmission  
    void SerializeHandlerData(std::shared_ptr<ChROSHandler> handler, double time, 
                             std::shared_ptr<ChROSIPCInterface> ipc_interface);
                             
    /// Serialize body handler data
    void SerializeBodyHandler(std::shared_ptr<class ChROSBodyHandler> handler, double time,
                             std::shared_ptr<ChROSIPCInterface> ipc_interface);
                             
    /// Serialize TF handler data  
    void SerializeTFHandler(std::shared_ptr<class ChROSTFHandler> handler, double time,
                           std::shared_ptr<ChROSIPCInterface> ipc_interface);
#endif
};

/// @}

}  // namespace ros
}  // namespace chrono

#endif
