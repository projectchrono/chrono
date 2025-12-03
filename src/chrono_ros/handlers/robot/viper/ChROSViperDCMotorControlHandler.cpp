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
#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include "chrono_ros_interfaces/msg/viper_wheel_id.hpp"

using std::placeholders::_1;

using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperDCMotorControlHandler::ChROSViperDCMotorControlHandler(double update_rate,
                                                                 std::shared_ptr<ViperDCMotorControl> driver,
                                                                 const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name), m_subscriber_setup_sent(false) {}

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

void ChROSViperDCMotorControlHandler::ApplyInputs(const chrono_ros_interfaces::msg::ViperDCMotorControl& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    m_msg = msg;
    
    // Immediately apply to driver in IPC mode
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

void ChROSViperDCMotorControlHandler::HandleIncomingMessage(const ipc::Message& msg) {
    const auto* viper_data = msg.GetPayload<ipc::ViperDCMotorControlData>();
    
    // Reconstruct the full message from simplified IPC data
    chrono_ros_interfaces::msg::ViperDCMotorControl full_msg;
    
    chrono_ros_interfaces::msg::ViperSteeringCommand steering_cmd;
    steering_cmd.angle = viper_data->steering_angle;
    steering_cmd.wheel_id = viper_data->steering_wheel_id;
    full_msg.driver_commands.steering_list.push_back(steering_cmd);
    
    full_msg.driver_commands.lifting = viper_data->lifting;
    full_msg.stall_torque.torque = viper_data->stall_torque;
    full_msg.stall_torque.wheel_id = viper_data->stall_torque_wheel_id;
    full_msg.no_load_speed.speed = viper_data->no_load_speed;
    full_msg.no_load_speed.wheel_id = viper_data->no_load_speed_wheel_id;
    
    ApplyInputs(full_msg);
}

std::vector<uint8_t> ChROSViperDCMotorControlHandler::GetSerializedData(double time) {
    // For subscribers in IPC mode:
    // Send topic name once to tell subprocess to create the subscriber
    // After that, return empty (subscriber receives data, doesn't publish)
    
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        
        // Serialize topic name to send to subprocess
        std::vector<uint8_t> data;
        data.resize(m_topic_name.size() + 1);
        std::memcpy(data.data(), m_topic_name.c_str(), m_topic_name.size() + 1);
        
        std::cout << "[MAIN PROCESS] Sending ViperDCMotorControl subscriber setup with topic: " 
                  << m_topic_name << std::endl;
        return data;
    }
    
    // After initial setup, return empty - subscriber receives data from ROS
    return std::vector<uint8_t>();
}

}  // namespace ros
}  // namespace chrono
