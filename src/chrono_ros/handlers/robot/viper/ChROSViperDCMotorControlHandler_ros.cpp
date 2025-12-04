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
// ROS subscriber handler for Viper DC Motor Control (subprocess only)
// This file contains ROS-specific code and is only linked into chrono_ros_node
//
// =============================================================================

#include <iostream>
#include <memory>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "chrono_ros_interfaces/msg/viper_dc_motor_control.hpp"

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler_ipc.h"

using namespace chrono::ros;
using namespace chrono::ros::ipc;

// Global subscriber and IPC channel for this handler
static rclcpp::Subscription<chrono_ros_interfaces::msg::ViperDCMotorControl>::SharedPtr g_viper_subscriber;
static IPCChannel* g_ipc_channel = nullptr;

/// Callback when ROS receives Viper motor control data
void OnViperDCMotorControlReceived(const chrono_ros_interfaces::msg::ViperDCMotorControl::SharedPtr msg) {
    std::cout << "[SUBPROCESS] ✓ Received ROS ViperDCMotorControl" << std::endl;
    
    if (!g_ipc_channel) {
        std::cerr << "[SUBPROCESS] ERROR: IPC channel not set for Viper handler!" << std::endl;
        return;
    }
    
    // For now, send a simplified version back to main process
    // The full message has complex nested structures, so we serialize key fields
    // You can extend this to send the complete message if needed
    
    ipc::ViperDCMotorControlData data;
    
    // Extract simplified data from message
    if (!msg->driver_commands.steering_list.empty()) {
        data.steering_angle = msg->driver_commands.steering_list[0].angle;
        data.steering_wheel_id = msg->driver_commands.steering_list[0].wheel_id;
    } else {
        data.steering_angle = 0.0;
        data.steering_wheel_id = -1;
    }
    
    data.lifting = msg->driver_commands.lifting;
    data.stall_torque = msg->stall_torque.torque;
    data.stall_torque_wheel_id = msg->stall_torque.wheel_id;
    data.no_load_speed = msg->no_load_speed.speed;
    data.no_load_speed_wheel_id = msg->no_load_speed.wheel_id;
    
    // Create IPC message and send back to main process
    // Use static to avoid repeated 64MB allocations
    static ipc::Message ipc_msg;
    
    // Re-initialize header and payload for each use
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::VIPER_DC_MOTOR_CONTROL, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));
    
    if (g_ipc_channel->SendMessage(ipc_msg)) {
        std::cout << "[SUBPROCESS] ✓ Sent ViperDCMotorControl via IPC to main process" << std::endl;
    } else {
        std::cerr << "[SUBPROCESS] ERROR: Failed to send Viper data via IPC!" << std::endl;
    }
}

/// Setup function called by subprocess to create ROS subscriber
/// @param data Serialized topic name from main process
/// @param data_size Size of data in bytes
/// @param node ROS node to create subscriber on
/// @param channel IPC channel for bidirectional communication
void SetupViperDCMotorControlSubscriber(const uint8_t* data, size_t data_size,
                                       rclcpp::Node::SharedPtr node,
                                       ipc::IPCChannel* channel) {
    // Extract topic name from data
    std::string topic_name;
    if (data_size > 0) {
        topic_name = std::string(reinterpret_cast<const char*>(data), data_size);
    } else {
        topic_name = "~/input/viper_dc_motor_control";  // Default
    }
    
    // Store IPC channel for callbacks
    g_ipc_channel = channel;
    
    // Create ROS subscriber
    g_viper_subscriber = node->create_subscription<chrono_ros_interfaces::msg::ViperDCMotorControl>(
        topic_name, 
        10,
        OnViperDCMotorControlReceived
    );
    
    std::cout << "[SUBPROCESS] Created Viper DC Motor Control subscriber on topic: " 
              << topic_name << std::endl;
}

// Register this handler with the registry using static initialization
// This happens automatically when the subprocess loads this compilation unit
CHRONO_ROS_REGISTER_HANDLER(VIPER_DC_MOTOR_CONTROL, SetupViperDCMotorControlSubscriber)
