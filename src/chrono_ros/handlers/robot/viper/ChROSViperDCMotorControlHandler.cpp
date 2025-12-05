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
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperDCMotorControlHandler::ChROSViperDCMotorControlHandler(double update_rate,
                                                                 std::shared_ptr<ViperDCMotorControl> driver,
                                                                 const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name), m_subscriber_setup_sent(false) {
    // Initialize inputs
    m_inputs.steering_angle = 0;
    m_inputs.steering_wheel_id = -1; // NONE
    m_inputs.lifting = 0;
    m_inputs.stall_torque = 0;
    m_inputs.stall_torque_wheel_id = -1; // NONE
    m_inputs.no_load_speed = 0;
    m_inputs.no_load_speed_wheel_id = -1; // NONE
}

bool ChROSViperDCMotorControlHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }
    return true;
}

std::vector<uint8_t> ChROSViperDCMotorControlHandler::GetSerializedData(double time) {
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        // Send topic name to subprocess to setup subscriber
        std::vector<uint8_t> data(m_topic_name.begin(), m_topic_name.end());
        data.push_back('\0'); // Null terminator
        return data;
    }
    return {};
}

void ChROSViperDCMotorControlHandler::HandleIncomingMessage(const ipc::Message& msg) {
    if (msg.header.type != ipc::MessageType::VIPER_DC_MOTOR_CONTROL) {
        return;
    }
    
    if (msg.header.payload_size != sizeof(ipc::ViperDCMotorControlData)) {
        std::cerr << "Received Viper control message with incorrect size" << std::endl;
        return;
    }
    
    auto data = reinterpret_cast<const ipc::ViperDCMotorControlData*>(msg.payload.get());
    ApplyInputs(*data);
}

void ChROSViperDCMotorControlHandler::ApplyInputs(const ipc::ViperDCMotorControlData& data) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_inputs = data;

    // Apply steering
    m_driver->SetSteering(m_inputs.steering_angle, static_cast<ViperWheelID>(m_inputs.steering_wheel_id));

    // Apply lifting
    m_driver->SetLifting(m_inputs.lifting);

    // Apply stall torque
    m_driver->SetMotorStallTorque(m_inputs.stall_torque, static_cast<ViperWheelID>(m_inputs.stall_torque_wheel_id));

    // Apply no load speed
    m_driver->SetMotorNoLoadSpeed(m_inputs.no_load_speed, static_cast<ViperWheelID>(m_inputs.no_load_speed_wheel_id));
}

}  // namespace ros
}  // namespace chrono

