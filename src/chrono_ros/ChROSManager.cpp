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
// Manager for the ROS handlers
//
// =============================================================================

#include "chrono_ros/ChROSManager.h"

#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono/core/ChTypes.h"

namespace chrono {
namespace ros {

ChROSManager::ChROSManager(const std::string& node_name) : m_interface(chrono_types::make_shared<ChROSInterface>(node_name)) {}

void ChROSManager::Initialize() {
    // Calls rclcpp::init()
    m_interface->Initialize();

    // Initialize the handlers. Print wanning and remove the handler if it fails to initialize.
    for (auto itr = m_handlers.begin(); itr != m_handlers.end();) {
        auto handler = *itr;
        if (!handler->Initialize(m_interface)) {
            std::cerr << "Failed to initialize ROS handler. Will remove handler and continue." << std::endl;
            itr = m_handlers.erase(itr);
        } else {
            itr++;
        }
    }
}

bool ChROSManager::Update(double time, double step) {
    for (auto handler : m_handlers)
        handler->Update(time, step);

    m_interface->SpinSome();

    return rclcpp::ok();
}

void ChROSManager::RegisterHandler(std::shared_ptr<ChROSHandler> handler) {
    m_handlers.push_back(handler);
}

}  // namespace ros
}  // namespace chrono