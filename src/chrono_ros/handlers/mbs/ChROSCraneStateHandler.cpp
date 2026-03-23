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
// Main-process implementation for the crane state handler.
// Extracts actuator length from the user-supplied callback and serializes it
// into a CraneStateData struct for IPC transmission.
//
// =============================================================================

#include "chrono_ros/handlers/mbs/ChROSCraneStateHandler.h"
#include "chrono_ros/handlers/mbs/ChROSHydraulicCraneHandler_ipc.h"

#include <cstring>

namespace chrono {
namespace ros {

ChROSCraneStateHandler::ChROSCraneStateHandler(double update_rate,
                                               StateCallback callback,
                                               const std::string& topic_name)
    : ChROSHandler(update_rate), m_callback(std::move(callback)), m_topic_name(topic_name) {
    m_buffer.resize(sizeof(ipc::CraneStateData));
}

bool ChROSCraneStateHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Nothing to do in the main process -- the subprocess creates the publisher.
    return true;
}

ipc::MessageType ChROSCraneStateHandler::GetMessageType() const {
    return ipc::MessageType::CRANE_STATE_DATA;
}

std::vector<uint8_t> ChROSCraneStateHandler::GetSerializedData(double time) {
    auto [s, sd] = m_callback();

    ipc::CraneStateData data{};
    strncpy(data.topic_name, m_topic_name.c_str(), sizeof(data.topic_name) - 1);
    data.s = s;
    data.sd = sd;

    m_buffer.resize(sizeof(ipc::CraneStateData));
    std::memcpy(m_buffer.data(), &data, sizeof(ipc::CraneStateData));
    return m_buffer;
}

}  // namespace ros
}  // namespace chrono
