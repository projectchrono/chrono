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
// Handler that drives a Viper ViperDCMotorControl from ROS
// (chrono_ros_interfaces/msg/ViperDCMotorControl).
//
// =============================================================================

#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSSubscription.h"

using namespace chrono::viper;

namespace chrono {
namespace ros {

// chrono_ros_interfaces/msg/ViperWheelID: V_LF=0..V_RB=3, V_UNDEFINED=4. The
// Chrono enum has only 0..3, so V_UNDEFINED is the "no specific wheel" sentinel.
static constexpr uint8_t VIPER_WHEEL_UNDEFINED = 4;

ChROSViperDCMotorControlHandler::ChROSViperDCMotorControlHandler(double update_rate,
                                                                 std::shared_ptr<ViperDCMotorControl> driver,
                                                                 const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name) {}

bool ChROSViperDCMotorControlHandler::Initialize(ChROSBridge& bridge) {
    // Callback fires on the simulation thread inside Update(), the same thread as
    // Tick(), so the stored commands need no lock.
    m_subscription = bridge.CreateSubscription(
        m_topic_name, "chrono_ros_interfaces/msg/ViperDCMotorControl",
        [this](const ChROSMessageView& msg) {
            m_steering.clear();
            const size_t n = msg.GetCount("driver_commands.steering_list");
            for (size_t i = 0; i < n; i++) {
                auto cmd = msg.GetMessage("driver_commands.steering_list", i);
                m_steering.push_back({cmd.GetDouble("angle"), static_cast<uint8_t>(cmd.GetUInt("wheel_id"))});
            }
            m_lifting = msg.GetDouble("driver_commands.lifting");
            m_stall_torque = msg.GetDouble("stall_torque.torque");
            m_stall_wheel = static_cast<uint8_t>(msg.GetUInt("stall_torque.wheel_id"));
            m_no_load_speed = msg.GetDouble("no_load_speed.speed");
            m_no_load_wheel = static_cast<uint8_t>(msg.GetUInt("no_load_speed.wheel_id"));
        });
    return true;
}

void ChROSViperDCMotorControlHandler::Tick(double time) {
    for (const auto& s : m_steering) {
        if (s.wheel_id != VIPER_WHEEL_UNDEFINED)
            m_driver->SetSteering(s.angle, static_cast<ViperWheelID>(s.wheel_id));
        else
            m_driver->SetSteering(s.angle);
    }
    m_driver->SetLifting(m_lifting);

    // wheel_id V_UNDEFINED (4) means "all wheels", mirroring the SetSteering
    // convention above. The driver has no all-wheels overload for torque/speed,
    // so loop the four wheels. Out-of-range ids are ignored (ViperWheelID only
    // defines 0..3, so this also avoids an out-of-bounds write in the driver).
    for (uint8_t id = 0; id < 4; id++) {
        if (m_stall_wheel == VIPER_WHEEL_UNDEFINED || m_stall_wheel == id)
            m_driver->SetMotorStallTorque(m_stall_torque, static_cast<ViperWheelID>(id));
        if (m_no_load_wheel == VIPER_WHEEL_UNDEFINED || m_no_load_wheel == id)
            m_driver->SetMotorNoLoadSpeed(m_no_load_speed, static_cast<ViperWheelID>(id));
    }
}

}  // namespace ros
}  // namespace chrono
