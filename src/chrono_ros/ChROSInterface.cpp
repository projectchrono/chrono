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
// The API interface between ROS/rclcpp and Chrono
//
// =============================================================================

#include "chrono_ros/ChROSInterface.h"

#include "chrono/core/ChTypes.h"

namespace chrono {
namespace ros {

ChROSInterface::ChROSInterface(const std::string node_name) : m_node_name(node_name) {}

void ChROSInterface::Initialize(rclcpp::NodeOptions options) {
    // Initialize only once
    // TODO: Is there any use case for argc and argv as parameters to rclcpp::init?
    if (!rclcpp::ok()) {
        std::cout << "Initializing rclcpp." << std::endl;
        rclcpp::init(0, 0);
    }

    // TODO: make options available to the user?
    m_executor = chrono_types::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    // TODO: Should we change the SignalHandlerOptions to None?
    m_node = std::make_shared<rclcpp::Node>(m_node_name, options);
    m_executor->add_node(m_node);

    std::cout << "Initialized ChROSInterface: " << m_node_name << "." << std::endl;
}

void ChROSInterface::SpinSome(std::chrono::nanoseconds max_duration) {
    // TODO: Should we use spin_all instead?
    if (rclcpp::ok()) {
        m_executor->spin_some(max_duration);
    }
}

}  // namespace ros
}  // namespace chrono