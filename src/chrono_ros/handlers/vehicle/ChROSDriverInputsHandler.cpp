// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Handler that drives a ChDriver from ROS (chrono_ros_interfaces/msg/DriverInputs).
//
// =============================================================================

#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSSubscription.h"

using namespace chrono::vehicle;

namespace chrono {
namespace ros {

ChROSDriverInputsHandler::ChROSDriverInputsHandler(std::shared_ptr<ChDriver> driver, const std::string& topic_name)
    : ChROSDriverInputsHandler(0, driver, topic_name) {}

ChROSDriverInputsHandler::ChROSDriverInputsHandler(double update_rate,
                                                   std::shared_ptr<ChDriver> driver,
                                                   const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name) {}

bool ChROSDriverInputsHandler::Initialize(ChROSBridge& bridge) {
    // The callback fires inside ChROSManager::Update() on the simulation thread
    // (invariant I5), the same thread as Tick(), so the stored inputs need no lock.
    m_subscription = bridge.CreateSubscription(
        m_topic_name, "chrono_ros_interfaces/msg/DriverInputs",
        [this](const ChROSMessageView& msg) {
            m_steering = msg.GetDouble("steering");
            m_throttle = msg.GetDouble("throttle");
            m_braking = msg.GetDouble("braking");
        });
    return true;
}

void ChROSDriverInputsHandler::Tick(double time) {
    m_driver->SetSteering(m_steering);
    m_driver->SetThrottle(m_throttle);
    m_driver->SetBraking(m_braking);
}

}  // namespace ros
}  // namespace chrono
