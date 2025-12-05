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
// ROS subscriber implementation for DriverInputsHandler  
// This file is ONLY compiled and linked in the subprocess (chrono_ros_node)
//
// BIDIRECTIONAL SUBSCRIBER PATTERN (subprocess side):
// 1. Main process calls handler->GetSerializedData() with topic name
// 2. Main process sends IPC message to trigger subscriber setup
// 3. Subprocess dispatcher receives IPC message
// 4. Dispatcher calls this setup function with topic name data
// 5. This function creates ROS subscriber with callback
// 6. When ROS messages arrive, callback packs IPC data and sends back to main process
// 7. Main process ChROSManager receives IPC message
// 8. Manager calls handler->HandleIncomingMessage() to apply data
//
// Key requirements:
// - Setup function signature: void(const std::vector<uint8_t>&, rclcpp::Node::SharedPtr, ipc::IPCChannel*)
// - Extract topic name from data vector (first call contains topic name)
// - Store ipc::IPCChannel* in static variable for callback to use
// - Callback creates IPC Message with same MessageType as handler
// - Send IPC message back to main process via channel->SendMessage()
// - Register with CHRONO_ROS_REGISTER_HANDLER(message_type, setup_function_name)
//
// BIDIRECTIONAL: Subscribes to ROS → sends IPC to main process
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler_ipc.h"
#include "chrono_ros_interfaces/msg/driver_inputs.hpp"

#include <cstring>
#include <iostream>

namespace chrono {
namespace ros {

/// Static storage for subscriber and IPC channel
static rclcpp::Subscription<chrono_ros_interfaces::msg::DriverInputs>::SharedPtr g_driver_subscriber;
static ipc::IPCChannel* g_ipc_channel = nullptr;

/// Callback when ROS message is received - sends via IPC back to main process
void OnDriverInputsReceived(const chrono_ros_interfaces::msg::DriverInputs::SharedPtr msg) {
    if (g_ipc_channel) {
        // Pack data into IPC message
        ipc::DriverInputsData data;
        data.steering = msg->steering;
        data.throttle = msg->throttle;
        data.braking = msg->braking;
        
        // Create IPC message
        // Use static to avoid repeated 64MB allocations
        static ipc::Message ipc_msg;
        
        // Re-initialize header and payload for each use
        ipc_msg.header = ipc::MessageHeader(ipc::MessageType::DRIVER_INPUTS, 0, sizeof(data), 0);
        std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));
        
        // Send to main process
        if (!g_ipc_channel->SendMessage(ipc_msg)) {
            std::cerr << "[SUBPROCESS] ✗ Failed to send DriverInputs via IPC" << std::endl;
        }
    } else {
        std::cerr << "[SUBPROCESS] ✗ No IPC channel available to send driver inputs!" << std::endl;
    }
}

/// Setup function called from subprocess dispatcher
/// This creates the ROS subscriber and stores the IPC channel for sending back
void SetupDriverInputsSubscriber(const uint8_t* data, size_t data_size,
                                 rclcpp::Node::SharedPtr node,
                                 ipc::IPCChannel* channel) {
    // Extract topic name from data
    std::string topic_name;
    if (data_size > 0) {
        topic_name = std::string(reinterpret_cast<const char*>(data), data_size);
    } else {
        topic_name = "~/input/driver_inputs";  // Default
    }
    
    // Store IPC channel for callback to use
    g_ipc_channel = channel;
    
    // Create subscriber (only once)
    if (!g_driver_subscriber) {
        g_driver_subscriber = node->create_subscription<chrono_ros_interfaces::msg::DriverInputs>(
            topic_name, 10, OnDriverInputsReceived);
    }
}

// Register this bidirectional handler
CHRONO_ROS_REGISTER_HANDLER(DRIVER_INPUTS, SetupDriverInputsSubscriber)

}  // namespace ros
}  // namespace chrono
