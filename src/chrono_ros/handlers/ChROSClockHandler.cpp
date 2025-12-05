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
// Clock publisher handler
//
// =============================================================================

#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

namespace chrono {
namespace ros {

ChROSClockHandler::ChROSClockHandler(double update_rate, const std::string& topic_name)
    : ChROSHandler(update_rate), m_topic_name(topic_name) {}

bool ChROSClockHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();
    if (!node) {
        return true;  // IPC mode - no node in main process
    }

    rclcpp::ClockQoS qos;
    m_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(m_topic_name, qos);
    return true;
}

std::vector<uint8_t> ChROSClockHandler::GetSerializedData(double time) {
    ChROSClockData data;
    data.time_seconds = time;
    
    std::vector<uint8_t> bytes(sizeof(ChROSClockData));
    std::memcpy(bytes.data(), &data, sizeof(ChROSClockData));
    return bytes;
}

}  // namespace ros
}  // namespace chrono