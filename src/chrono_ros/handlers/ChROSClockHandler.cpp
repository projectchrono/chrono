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
// Clock publisher handler.
//
// =============================================================================

#include "chrono_ros/handlers/ChROSClockHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSQoS.h"

namespace chrono {
namespace ros {

ChROSClockHandler::ChROSClockHandler(double update_rate, const std::string& topic_name)
    : ChROSHandler(update_rate), m_topic_name(topic_name) {}

bool ChROSClockHandler::Initialize(ChROSBridge& bridge) {
    m_publisher = bridge.CreatePublisher(m_topic_name, "rosgraph_msgs/msg/Clock", ChROSQoS::Clock());
    return true;
}

void ChROSClockHandler::Tick(double time) {
    auto msg = m_publisher->NewMessage();
    msg.SetTime("clock", time);  // rosgraph_msgs/Clock has a single builtin_interfaces/Time field
    m_publisher->Publish(msg);
}

}  // namespace ros
}  // namespace chrono
