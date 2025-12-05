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
// The API interface between ROS/rclcpp and Chrono
//
// =============================================================================

#include "chrono_ros/ChROSInterface.h"

#include "chrono/core/ChTypes.h"
#include <iostream>

namespace chrono {
namespace ros {

ChROSInterface::ChROSInterface(const std::string node_name) : m_node_name(node_name) {}

void ChROSInterface::Initialize(rclcpp::NodeOptions options) {
    // Initialize only once
    if (!rclcpp::ok()) {
        rclcpp::init(0, 0);
    }

    m_executor = chrono_types::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    m_node = std::make_shared<rclcpp::Node>(m_node_name, options);
    m_executor->add_node(m_node);

    std::cout << "Initialized ChROSInterface: " << m_node_name << "." << std::endl;
}

void ChROSInterface::SpinSome(std::chrono::nanoseconds max_duration) {
    if (rclcpp::ok()) {
        m_executor->spin_some(max_duration);
    }
}

}  // namespace ros
}  // namespace chrono