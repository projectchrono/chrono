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
// Author: Aaron Young
// =============================================================================
//
// Handler for interfacing a ChDriver to ROS
//
// =============================================================================

#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

using namespace chrono::vehicle;

using std::placeholders::_1;

namespace chrono {
namespace ros {

ChROSDriverInputsHandler::ChROSDriverInputsHandler(std::shared_ptr<ChDriver> driver, const std::string& topic_name)
    : ChROSDriverInputsHandler(0, driver, topic_name) {}

ChROSDriverInputsHandler::ChROSDriverInputsHandler(double update_rate,
                                                   std::shared_ptr<ChDriver> driver,
                                                   const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name), m_inputs({0, 0, 0, 0}) {}

bool ChROSDriverInputsHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_subscription = node->create_subscription<chrono_ros_interfaces::msg::DriverInputs>(
        m_topic_name, 1, std::bind(&ChROSDriverInputsHandler::Callback, this, _1));

    return true;
}

void ChROSDriverInputsHandler::Callback(const chrono_ros_interfaces::msg::DriverInputs& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_inputs.m_steering = msg.steering;
    m_inputs.m_throttle = msg.throttle;
    m_inputs.m_braking = msg.braking;
}

void ChROSDriverInputsHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_driver->SetSteering(m_inputs.m_steering);
    m_driver->SetThrottle(m_inputs.m_throttle);
    m_driver->SetBraking(m_inputs.m_braking);
}

}  // namespace ros
}  // namespace chrono
