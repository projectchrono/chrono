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
// Handler that publishes transform (tf) information on /tf.
//
// =============================================================================

#include "chrono_ros/handlers/ChROSTFHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSPublisher.h"

#ifdef CHRONO_SENSOR
    #include "chrono_sensor/sensors/ChSensor.h"
#endif

#include <iostream>

namespace chrono {
namespace ros {

ChROSTFHandler::ChROSTFHandler(double update_rate) : ChROSHandler(update_rate) {}

bool ChROSTFHandler::Initialize(ChROSBridge& bridge) {
    m_publisher = bridge.CreatePublisher("/tf", "tf2_msgs/msg/TFMessage");
    return true;
}

void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent,
                                  const std::string& parent_frame_id,
                                  std::shared_ptr<chrono::ChBody> child,
                                  const std::string& child_frame_id) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: parent body is null" << std::endl;
        return;
    }
    if (!child) {
        std::cerr << "ChROSTFHandler::AddTransform: child body is null" << std::endl;
        return;
    }

    ChBodyTransform parent_tf(parent, !parent_frame_id.empty() ? parent_frame_id : parent->GetName());
    ChBodyTransform child_tf(child, !child_frame_id.empty() ? child_frame_id : child->GetName());
    m_transforms.emplace_back(parent_tf, child_tf);
}

void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent,
                                  const std::string& parent_frame_id,
                                  chrono::ChFrame<double> child_frame,
                                  const std::string& child_frame_id) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: parent body is null" << std::endl;
        return;
    }

    ChBodyTransform parent_tf(parent, !parent_frame_id.empty() ? parent_frame_id : parent->GetName());
    ChFrameTransform child_tf(child_frame, child_frame_id);
    m_transforms.emplace_back(parent_tf, child_tf);
}

#ifdef CHRONO_SENSOR
void ChROSTFHandler::AddSensor(std::shared_ptr<chrono::sensor::ChSensor> sensor,
                              const std::string& parent_frame_id,
                              const std::string& child_frame_id) {
    if (!sensor) {
        std::cerr << "ChROSTFHandler::AddSensor: sensor is null" << std::endl;
        return;
    }
    // The sensor's offset pose is the child frame, expressed in its parent body.
    AddTransform(sensor->GetParent(), parent_frame_id, sensor->GetOffsetPose(), child_frame_id);
}
#endif

#ifdef CHRONO_HAS_URDF
void ChROSTFHandler::AddURDF(chrono::parsers::ChParserURDF& parser) {
    auto model = parser.GetModelTree();
    std::vector<urdf::LinkSharedPtr> links;
    model->getLinks(links);
    for (const auto& link : links) {
        auto joint = link->parent_joint;
        if (!joint)
            continue;  // root link has no parent joint
        auto parent = parser.GetChBody(joint->parent_link_name);
        auto child = parser.GetChBody(joint->child_link_name);
        AddTransform(parent, joint->parent_link_name, child, joint->child_link_name);
    }
}
#endif

void ChROSTFHandler::Tick(double time) {
    auto msg = m_publisher->NewMessage();

    for (const auto& [parent_tf, child_tf] : m_transforms) {
        // The parent is always a body transform.
        const auto& [parent_body, parent_frame_id] = std::get<ChBodyTransform>(parent_tf);

        chrono::ChFrame<> child_to_parent;
        std::string child_frame_id;
        if (std::holds_alternative<ChBodyTransform>(child_tf)) {
            const auto& [child_body, cfid] = std::get<ChBodyTransform>(child_tf);
            child_to_parent = parent_body->GetFrameRefToAbs().GetInverse() * child_body->GetFrameRefToAbs();
            child_frame_id = cfid;
        } else {
            const auto& [cframe, cfid] = std::get<ChFrameTransform>(child_tf);
            child_to_parent = cframe;
            child_frame_id = cfid;
        }

        auto tf = msg.AppendMessage("transforms");
        tf.SetString("header.frame_id", parent_frame_id);
        tf.SetTime("header.stamp", time);
        tf.SetString("child_frame_id", child_frame_id);

        const auto& pos = child_to_parent.GetPos();
        tf.SetDouble("transform.translation.x", pos.x());
        tf.SetDouble("transform.translation.y", pos.y());
        tf.SetDouble("transform.translation.z", pos.z());

        const auto& q = child_to_parent.GetRot();
        tf.SetDouble("transform.rotation.x", q.e1());
        tf.SetDouble("transform.rotation.y", q.e2());
        tf.SetDouble("transform.rotation.z", q.e3());
        tf.SetDouble("transform.rotation.w", q.e0());
    }

    m_publisher->Publish(msg);
}

}  // namespace ros
}  // namespace chrono
