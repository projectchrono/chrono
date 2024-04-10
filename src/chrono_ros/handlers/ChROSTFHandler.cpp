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
// Handler responsible for publishing transform (tf) information
//
// =============================================================================

#include "chrono_ros/handlers/ChROSTFHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace chrono {
namespace ros {

ChROSTFHandler::ChROSTFHandler(double update_rate) : ChROSHandler(update_rate) {}

bool ChROSTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    return true;
}

void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent, std::shared_ptr<chrono::ChBody> child) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: Parent body is null" << std::endl;
        return;
    } else if (!child) {
        std::cerr << "ChROSTFHandler::AddTransform: Child body is null" << std::endl;
        return;
    }

    m_transforms.push_back(std::make_pair(parent, child));
}

void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent,
                                  chrono::ChFrame<double> child_frame,
                                  const std::string& child_frame_id) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: Parent body is null" << std::endl;
        return;
    }

    ChFrameTransform child = std::make_pair(child_frame, child_frame_id);
    m_transforms.push_back(std::make_pair(parent, child));
}

#ifdef CHRONO_PARSERS_URDF
void ChROSTFHandler::AddURDF(chrono::parsers::ChParserURDF& parser) {
    auto model = parser.GetModelTree();

    std::vector<urdf::LinkSharedPtr> links;
    model->getLinks(links);

    for (const auto& link : links) {
        auto joint = link->parent_joint;
        if (!joint)
            continue;

        auto parent = parser.GetChBody(joint->parent_link_name);
        auto child = parser.GetChBody(joint->child_link_name);
        AddTransform(parent, child);
    }
}
#endif

#ifdef CHRONO_SENSOR
void ChROSTFHandler::AddSensor(std::shared_ptr<chrono::sensor::ChSensor> sensor, const std::string& frame_id) {
    if (!sensor) {
        std::cerr << "ChROSTFHandler::AddSensor: Sensor is null" << std::endl;
        return;
    }

    auto parent = sensor->GetParent();
    auto child_frame = sensor->GetOffsetPose();
    auto child_frame_id = frame_id.empty() ? sensor->GetName() : frame_id;
    AddTransform(parent, child_frame, child_frame_id);
}
#endif

geometry_msgs::msg::TransformStamped CreateTransformStamped(chrono::ChFrame<> local_to_parent,
                                                            const std::string& parent_frame_id,
                                                            const std::string& child_frame_id,
                                                            double time) {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    tf_msg.header.frame_id = parent_frame_id;
    tf_msg.child_frame_id = child_frame_id;

    auto pos = local_to_parent.GetPos();
    tf_msg.transform.translation.x = pos.x();
    tf_msg.transform.translation.y = pos.y();
    tf_msg.transform.translation.z = pos.z();

    auto quat = local_to_parent.GetRot();
    tf_msg.transform.rotation.w = quat.e0();
    tf_msg.transform.rotation.x = quat.e1();
    tf_msg.transform.rotation.y = quat.e2();
    tf_msg.transform.rotation.z = quat.e3();

    return tf_msg;
}

void ChROSTFHandler::Tick(double time) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    for (auto const& [parent, child] : m_transforms) {
        chrono::ChFrame<> child_to_parent;
        std::string child_frame_id;
        if (std::holds_alternative<std::shared_ptr<chrono::ChBody>>(child)) {
            auto child_body = std::get<std::shared_ptr<chrono::ChBody>>(child);
            child_to_parent = parent->GetFrameRefToAbs().GetInverse() * child_body->GetFrameRefToAbs();
            child_frame_id = child_body->GetName();
        } else {
            auto frame_pair = std::get<ChFrameTransform>(child);
            child_to_parent = frame_pair.first;
            child_frame_id = frame_pair.second;
        }

        auto tf_msg = CreateTransformStamped(child_to_parent, parent->GetName(), child_frame_id, time);
        transforms.push_back(tf_msg);
    }

    m_tf_broadcaster->sendTransform(transforms);
}

}  // namespace ros
}  // namespace chrono
