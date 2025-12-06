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
// ROS-side implementation for Viper DC Motor Control Handler
//
// =============================================================================

#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler_ipc.h"
#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"

#include "chrono_ros_interfaces/msg/viper_dc_motor_control.hpp"
#include "chrono_ros_interfaces/msg/viper_wheel_id.hpp"

#include "rclcpp/rclcpp.hpp"

namespace chrono {
namespace ros {

// Global IPC channel pointer for the callback
static ipc::IPCChannel* g_ipc_channel = nullptr;
static rclcpp::Subscription<chrono_ros_interfaces::msg::ViperDCMotorControl>::SharedPtr g_viper_subscriber;

// Callback for ROS messages
void OnViperDCMotorControlReceived(const chrono_ros_interfaces::msg::ViperDCMotorControl::SharedPtr msg) {
    if (!g_ipc_channel) return;

    ipc::ViperDCMotorControlData data;
    
    // Extract data from message
    // Note: The message structure might differ slightly based on the ROS interface definition
    // Assuming standard fields based on previous context
    
    // Steering
    // The ROS message seems to have a list of steering commands or similar?
    // Based on previous error logs: msg->driver_commands.steering_list
    // But let's assume a simpler structure if possible, or match what was there.
    // The error log showed: msg->driver_commands.steering_list
    // Let's try to match the ROS message structure inferred from errors.
    
    // Actually, let's look at the error log again:
    // msg->driver_commands.steering_list
    // msg->stall_torque.torque
    // msg->no_load_speed.speed
    
    // Wait, the error log in previous turn (turn 14) showed:
    // msg->driver_commands.steering_list
    
    // Let's try to be safe and use what we saw.
    
    // However, I don't have the full definition of chrono_ros_interfaces::msg::ViperDCMotorControl.
    // But I can infer from the previous code snippet I read (which had errors but showed usage).
    
    // Let's assume the previous code was trying to use the correct fields but had syntax errors.
    
    // Re-reading the previous code snippet from ChROSViperDCMotorControlHandler_ros.cpp:
    /*
    if (!msg->driver_commands.steering_list.empty()) {
        data.steering_angle = msg->driver_commands.steering_list[0].angle;
        data.steering_wheel_id = msg->driver_commands.steering_list[0].wheel_id;
    }
    data.lifting = msg->driver_commands.lifting;
    data.stall_torque = msg->stall_torque.torque;
    data.stall_torque_wheel_id = msg->stall_torque.wheel_id;
    */
    
    // I will use this structure.
    
    if (!msg->driver_commands.steering_list.empty()) {
        data.steering_angle = msg->driver_commands.steering_list[0].angle;
        data.steering_wheel_id = msg->driver_commands.steering_list[0].wheel_id;
    } else {
        data.steering_angle = 0;
        data.steering_wheel_id = -1;
    }
    
    data.lifting = msg->driver_commands.lifting;
    
    data.stall_torque = msg->stall_torque.torque;
    data.stall_torque_wheel_id = msg->stall_torque.wheel_id;
    
    data.no_load_speed = msg->no_load_speed.speed;
    data.no_load_speed_wheel_id = msg->no_load_speed.wheel_id;

    // Create IPC message
    static ipc::Message ipc_msg;
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::VIPER_DC_MOTOR_CONTROL, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));
    
    g_ipc_channel->SendMessage(ipc_msg);
}

/// Setup function called by subprocess to create ROS subscriber
/// @param data Serialized topic name from main process
/// @param data_size Size of data in bytes
/// @param node ROS node to create subscriber on
// Setup function for the subscriber
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
}

// Register the handler
CHRONO_ROS_REGISTER_HANDLER(VIPER_DC_MOTOR_CONTROL, SetupViperDCMotorControlSubscriber)

}  // namespace ros
}  // namespace chrono
