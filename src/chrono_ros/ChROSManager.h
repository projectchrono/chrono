// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// ChROSManager: user entry point of Chrono::ROS.
//
//   auto manager = chrono_types::make_shared<ChROSManager>();
//   manager->RegisterHandler(...);          // built-in or your own handlers
//   manager->Initialize();                  // launches the bridge node
//   while (...) {
//       manager->Update(time, step);        // every simulation step
//       sys.DoStepDynamics(step);
//   }
//
// Update() drains inbound messages (firing subscription callbacks on this
// thread) and ticks handlers at their sim-time rates. The simulation is the
// time master and never blocks on ROS; see the module documentation for the
// synchronization model and the lock-step option
// (ChROSSubscription::WaitForMessage).
//
// =============================================================================

#ifndef CH_ROS_MANAGER_H
#define CH_ROS_MANAGER_H

#include "chrono_ros/ChApiROS.h"

#include <memory>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

class ChROSBridge;
class ChROSHandler;

/// @addtogroup ros_core
/// @{

class CH_ROS_API ChROSManager {
  public:
    /// One manager = one bridge node. Multiple managers (each with a unique
    /// node name) may coexist in one simulation.
    explicit ChROSManager(const std::string& node_name = "chrono_ros_node");
    ~ChROSManager();

    ChROSManager(const ChROSManager&) = delete;
    ChROSManager& operator=(const ChROSManager&) = delete;

    /// Shared-memory capacity override; see ChROSBridge::SetChannelCapacity.
    /// Must be called before Initialize().
    void SetChannelCapacity(size_t sim_to_node_bytes, size_t node_to_sim_bytes);

    /// Launch the bridge node and initialize every registered handler, in
    /// registration order. Throws std::runtime_error with an actionable
    /// message on any failure (fail-fast: a misconfigured pathway never
    /// degrades into silent absence of data).
    void Initialize();

    /// Advance the ROS side by one simulation step: deliver inbound messages
    /// to subscription callbacks and tick the handlers that are due.
    /// @param time current simulation time (seconds)
    /// @param step simulation step size just taken (seconds)
    /// @return false if the bridge node died (the simulation may continue
    ///         without ROS, or treat this as fatal - caller's choice)
    bool Update(double time, double step);

    /// Register a handler. Allowed before Initialize() (initialized with the
    /// manager) or after (initialized immediately).
    void RegisterHandler(std::shared_ptr<ChROSHandler> handler);

    /// Access the bridge for direct publisher/subscription creation and
    /// type discovery (DescribeType).
    std::shared_ptr<ChROSBridge> GetBridge() { return m_bridge; }

  private:
    std::shared_ptr<ChROSBridge> m_bridge;
    std::vector<std::shared_ptr<ChROSHandler>> m_handlers;
    bool m_initialized = false;
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif
