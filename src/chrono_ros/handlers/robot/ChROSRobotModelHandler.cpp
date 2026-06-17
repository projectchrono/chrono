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
// Handler that publishes a robot model (URDF) on /robot_description for RViz.
//
// =============================================================================

#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSQoS.h"

namespace chrono {
namespace ros {

// update_rate 0 -> Tick runs every step, but we publish exactly once (the model
// is static) and rely on the latched QoS to deliver to late subscribers. (9.0
// passed a huge update_rate, which actually republished the string every step;
// this is the efficient equivalent.)
ChROSRobotModelHandler::ChROSRobotModelHandler(const std::string& robot_model, const std::string& topic_name)
    : ChROSHandler(0), m_topic_name(topic_name), m_robot_model(robot_model) {}

bool ChROSRobotModelHandler::Initialize(ChROSBridge& bridge) {
    m_publisher = bridge.CreatePublisher(m_topic_name, "std_msgs/msg/String", ChROSQoS::Latched());
    return true;
}

void ChROSRobotModelHandler::Tick(double time) {
    if (m_published)
        return;

    auto msg = m_publisher->NewMessage();
    msg.SetString("data", m_robot_model);
    if (m_publisher->Publish(msg))
        m_published = true;  // static model; the latched QoS serves late joiners
}

}  // namespace ros
}  // namespace chrono
