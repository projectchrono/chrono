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
// Handler that publishes the full hydraulic actuator state (force, valve
// position, cylinder pressures, and actuation reference) to a ROS topic.
//
// This is a PUBLISHER handler (Chrono -> ROS).  Like the crane state handler,
// it takes a user-supplied callback rather than coupling directly to the
// Actuator class, keeping the handler reusable.
//
// =============================================================================

#ifndef CH_ROS_ACTUATOR_STATE_HANDLER_H
#define CH_ROS_ACTUATOR_STATE_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// Snapshot of the hydraulic actuator's state at a single time step.
struct ActuatorSnapshot {
    double force;           ///< Actuator force [N]
    double valve_position;  ///< Directional valve spool position [-]
    double pressure_0;      ///< Chamber 0 pressure [Pa]
    double pressure_1;      ///< Chamber 1 pressure [Pa]
    double Uref;            ///< Actuation reference input [-]
};

/// Publishes hydraulic actuator state to ROS.
///
/// Data flow:
///   Callback → GetSerializedData() → IPC → Subprocess → ROS topic
///
/// Required companion files:
///   - ChROSHydraulicCraneHandler_ipc.h       (shared IPC struct)
///   - ChROSActuatorStateHandler_ros.cpp       (subprocess publisher)
class CH_ROS_API ChROSActuatorStateHandler : public ChROSHandler {
  public:
    /// Callback signature: returns a snapshot of the actuator state.
    using StateCallback = std::function<ActuatorSnapshot()>;

    /// Construct an actuator state publisher.
    /// @param update_rate  Publishing rate in Hz (0 = every tick)
    /// @param callback     Lambda or function returning the actuator snapshot
    /// @param topic_name   ROS topic on which to publish the state
    ChROSActuatorStateHandler(double update_rate,
                              StateCallback callback,
                              const std::string& topic_name = "~/actuator/state");

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
