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
// ROS publishing implementation for ClockHandler
// This file is ONLY compiled and linked in the subprocess (chrono_ros_node)
//
// PUBLISHER PATTERN (subprocess side):
// 1. Main process calls handler->GetSerializedData(time)
// 2. Main process sends IPC message with serialized data
// 3. Subprocess dispatcher receives IPC message
// 4. Dispatcher looks up handler by MessageType in registry
// 5. Dispatcher calls this function with deserialized data
// 6. This function creates ROS publisher (lazy init) and publishes message
//
// Key requirements:
// - Function signature: void(const std::vector<uint8_t>&, rclcpp::Node::SharedPtr, ipc::IPCChannel*)
// - Deserialize data using std::memcpy (must match IPC struct exactly)
// - Use static variables for publishers (lazy initialization on first call)
// - Register with CHRONO_ROS_REGISTER_HANDLER(message_type, function_name)
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "rosgraph_msgs/msg/clock.hpp"

#include <cstring>

namespace chrono {
namespace ros {

/// ROS publishing function for clock handler
/// This is called in the subprocess when clock data arrives via IPC
void PublishClockToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    // Channel not needed for publishers (only for subscribers sending back)
    (void)channel;
    // Deserialize the data
    if (data_size != sizeof(ChROSClockData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid clock data size: %zu, expected %zu", 
                     data_size, sizeof(ChROSClockData));
        return;
    }
    
    ChROSClockData clock_data;
    std::memcpy(&clock_data, data, sizeof(ChROSClockData));
    
    // Create publisher if needed (lazy initialization with static variable)
    static rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher;
    if (!publisher) {
        rclcpp::ClockQoS qos;
        publisher = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
        RCLCPP_INFO(node->get_logger(), "Created clock publisher on /clock");
    }
    
    // Create and publish ROS message
    rosgraph_msgs::msg::Clock msg;
    msg.clock = ChROSHandlerUtilities::GetROSTimestamp(clock_data.time_seconds);
    publisher->publish(msg);
}

// Register this handler's publish function with the registry
// This happens automatically when the subprocess starts
CHRONO_ROS_REGISTER_HANDLER(CLOCK_DATA, PublishClockToROS)

}  // namespace ros
}  // namespace chrono