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
// Base class for ROS handlers: rate-scheduled callbacks that move data
// between the Chrono simulation and ROS topics through the ChROSBridge.
//
// Subclass (in C++ or Python), create publishers/subscriptions in
// Initialize(), fill and publish messages in Tick(). No bridge or subprocess
// code is ever required - see demo_ROS_custom_handler and the module
// documentation.
//
// =============================================================================

#ifndef CH_ROS_HANDLER_H
#define CH_ROS_HANDLER_H

#include "chrono_ros/ChApiROS.h"

#include <cstdint>

namespace chrono {
namespace ros {

class ChROSBridge;
class ChROSManager;

/// @addtogroup ros_handlers
/// @{

class CH_ROS_API ChROSHandler {
  public:
    /// @param update_rate handler frequency relative to *simulation* time, in
    /// Hz. A rate of 0 ticks on every simulation step.
    explicit ChROSHandler(double update_rate);
    virtual ~ChROSHandler() = default;

    /// Create this handler's publishers/subscriptions here. Called once from
    /// ChROSManager::Initialize(), after the bridge is up. Return false (or
    /// throw) to abort initialization.
    virtual bool Initialize(ChROSBridge& bridge) = 0;

    /// Produce/consume data. Called from ChROSManager::Update() on the
    /// simulation thread whenever this handler is due per its update rate.
    /// @param time current simulation time in seconds
    virtual void Tick(double time) = 0;

    /// Configured update rate in Hz (sim time); 0 means every step.
    double GetUpdateRate() const { return m_update_rate; }
    /// Number of times Tick() has been called.
    uint64_t GetTickCount() const { return m_tick_count; }

  private:
    friend class ChROSManager;

    /// Rate scheduling: accumulate simulation time and invoke Tick when due.
    void Advance(double time, double step);

    const double m_update_rate;
    uint64_t m_tick_count = 0;
    double m_time_elapsed_since_last_tick = 0;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
