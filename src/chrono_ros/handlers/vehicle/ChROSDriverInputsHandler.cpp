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
// Author: Aaron Young, Patrick Chen
// =============================================================================
//
// Handler for interfacing a ChDriver to ROS
//
// =============================================================================

#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

using namespace chrono::vehicle;

namespace chrono {
namespace ros {

ChROSDriverInputsHandler::ChROSDriverInputsHandler(std::shared_ptr<ChDriver> driver, const std::string& topic_name)
    : ChROSDriverInputsHandler(0, driver, topic_name) {}

ChROSDriverInputsHandler::ChROSDriverInputsHandler(double update_rate,
                                                   std::shared_ptr<ChDriver> driver,
                                                   const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name), 
      m_inputs({0, 0, 0, 0}), m_applied_inputs({0, 0, 0, 0}), m_subscriber_setup_sent(false) {}

bool ChROSDriverInputsHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }
    return true;
}

void ChROSDriverInputsHandler::ApplyInputs(double steering, double throttle, double braking) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    m_inputs.m_steering = steering;
    m_inputs.m_throttle = throttle;
    m_inputs.m_braking = braking;
    
    // Immediately apply to driver in IPC mode
    m_driver->SetSteering(m_inputs.m_steering);
    m_driver->SetThrottle(m_inputs.m_throttle);
    m_driver->SetBraking(m_inputs.m_braking);
    
    m_applied_inputs = m_inputs;
}

void ChROSDriverInputsHandler::HandleIncomingMessage(const ipc::Message& msg) {
    const auto* data = msg.GetPayload<ipc::DriverInputsData>();
    ApplyInputs(data->steering, data->throttle, data->braking);
}

std::vector<uint8_t> ChROSDriverInputsHandler::GetSerializedData(double time) {
    // For subscribers in IPC mode:
    // Send topic name once to tell subprocess to create the subscriber
    // After that, return empty (subscriber receives data, doesn't publish)
    
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        
        // Serialize topic name to send to subprocess
        std::vector<uint8_t> data(m_topic_name.begin(), m_topic_name.end());
        return data;
    }
    
    return {};
}

}  // namespace ros
}  // namespace chrono
