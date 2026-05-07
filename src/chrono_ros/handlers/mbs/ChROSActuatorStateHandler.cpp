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
// Main-process implementation for the actuator state handler.
// Extracts force, valve position, and cylinder pressures through the
// user-supplied callback and serializes into an ActuatorStateData struct.
//
// =============================================================================

#include "chrono_ros/handlers/mbs/ChROSActuatorStateHandler.h"
#include "chrono_ros/handlers/mbs/ChROSHydraulicCraneHandler_ipc.h"

#include <cstring>

namespace chrono {
namespace ros {

ChROSActuatorStateHandler::ChROSActuatorStateHandler(double update_rate,
                                                     StateCallback callback,
                                                     const std::string& topic_name)
    : ChROSHandler(update_rate), m_callback(std::move(callback)), m_topic_name(topic_name) {
    m_buffer.resize(sizeof(ipc::ActuatorStateData));
}

bool ChROSActuatorStateHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    return true;
}

ipc::MessageType ChROSActuatorStateHandler::GetMessageType() const {
    return ipc::MessageType::ACTUATOR_STATE_DATA;
}

std::vector<uint8_t> ChROSActuatorStateHandler::GetSerializedData(double time) {
    auto snap = m_callback();

    ipc::ActuatorStateData data{};
    strncpy(data.topic_name, m_topic_name.c_str(), sizeof(data.topic_name) - 1);
    data.force = snap.force;
    data.valve_position = snap.valve_position;
    data.pressure_0 = snap.pressure_0;
    data.pressure_1 = snap.pressure_1;
    data.Uref = snap.Uref;

    m_buffer.resize(sizeof(ipc::ActuatorStateData));
    std::memcpy(m_buffer.data(), &data, sizeof(ipc::ActuatorStateData));
    return m_buffer;
}

}  // namespace ros
}  // namespace chrono
