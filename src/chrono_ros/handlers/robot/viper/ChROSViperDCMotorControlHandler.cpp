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
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "chrono_ros_interfaces/msg/viper_wheel_id.hpp"

using std::placeholders::_1;

using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperDCMotorControlHandler::ChROSViperDCMotorControlHandler(double update_rate,
                                                                 std::shared_ptr<ViperDCMotorControl> driver,
                                                                 const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name) {}

bool ChROSViperDCMotorControlHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_subscription = node->create_subscription<chrono_ros_interfaces::msg::ViperDCMotorControl>(
        m_topic_name, 1, std::bind(&ChROSViperDCMotorControlHandler::Callback, this, _1));

    return true;
}

void ChROSViperDCMotorControlHandler::Callback(const chrono_ros_interfaces::msg::ViperDCMotorControl& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_msg = msg;
}

void ChROSViperDCMotorControlHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);

    for (auto steering_command : m_msg.driver_commands.steering_list) {
        if (steering_command.wheel_id != chrono_ros_interfaces::msg::ViperWheelID::V_UNDEFINED)
            m_driver->SetSteering(steering_command.angle,
                                  static_cast<chrono::viper::ViperWheelID>(steering_command.wheel_id));
        else
            m_driver->SetSteering(steering_command.angle);
    }

    m_driver->SetLifting(m_msg.driver_commands.lifting);
    m_driver->SetMotorStallTorque(m_msg.stall_torque.torque,
                                  static_cast<chrono::viper::ViperWheelID>(m_msg.stall_torque.wheel_id));
    m_driver->SetMotorNoLoadSpeed(m_msg.no_load_speed.speed,
                                  static_cast<chrono::viper::ViperWheelID>(m_msg.no_load_speed.wheel_id));
}

}  // namespace ros
}  // namespace chrono
