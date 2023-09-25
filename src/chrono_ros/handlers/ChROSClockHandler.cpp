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

    rclcpp::ClockQoS qos;
    m_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(m_topic_name, qos);

    return true;
}

void ChROSClockHandler::Tick(double time) {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono