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
// Authors: Patrick Chen
// =============================================================================
//
// Handler that publishes the kinematic state of a crane mechanism (actuator
// length and its rate of change) to a ROS topic via IPC.
//
// This is a PUBLISHER handler (Chrono -> ROS).  The main process extracts
// the displacement information each tick, and the ROS subprocess publishes it
// as a std_msgs/Float64MultiArray message.
//
// =============================================================================

#ifndef CH_ROS_CRANE_STATE_HANDLER_H
#define CH_ROS_CRANE_STATE_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include <functional>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// Publishes crane actuator displacement (length and rate) to ROS.
///
/// The handler is intentionally decoupled from any specific crane class so that
/// it can wrap *any* mechanism that provides an actuator-length query.  The user
/// supplies a callback that returns (s, sd) at the current simulation state.
///
/// Data flow:
///   Callback → GetSerializedData() → IPC → Subprocess → ROS topic
///
/// Required companion files:
///   - ChROSHydraulicCraneHandler_ipc.h   (shared IPC struct)
///   - ChROSCraneStateHandler_ros.cpp      (subprocess publisher)
class CH_ROS_API ChROSCraneStateHandler : public ChROSHandler {
  public:
    /// Callback signature: returns (actuator_length, actuator_length_rate).
    using StateCallback = std::function<std::pair<double, double>()>;

    /// Construct a crane state publisher.
    /// @param update_rate  Publishing rate in Hz (0 = every tick)
    /// @param callback     Lambda or function that queries the crane for (s, sd)
    /// @param topic_name   ROS topic on which to publish the state
    ChROSCraneStateHandler(double update_rate,
                           StateCallback callback,
                           const std::string& topic_name = "~/crane/state");

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override;
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    StateCallback m_callback;
    const std::string m_topic_name;
    std::vector<uint8_t> m_buffer;  ///< Reusable serialization buffer
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
