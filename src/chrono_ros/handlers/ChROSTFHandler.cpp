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
// Handler responsible for publishing transform (tf) information
//
// =============================================================================

#include "chrono_ros/handlers/ChROSTFHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace chrono {
namespace ros {

ChROSTFHandler::ChROSTFHandler(double update_rate) : ChROSHandler(update_rate) {}

bool ChROSTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();
    if (!node) {
        return true;  // IPC mode - no node in main process
    }

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    return true;
}

std::vector<uint8_t> ChROSTFHandler::GetSerializedData(double time) {
    // Compute all transforms first
    std::vector<ipc::TFTransform> transform_data;
    transform_data.reserve(m_transforms.size());
    
    for (const auto& [parent_tf, child_tf] : m_transforms) {
        chrono::ChFrame<> child_to_parent;
        std::string parent_id;
        std::string child_id;

        const auto& [parent_body, parent_frame_id] = std::get<ChBodyTransform>(parent_tf);
        parent_id = parent_frame_id;

        if (std::holds_alternative<ChBodyTransform>(child_tf)) {
            const auto& [child_body, child_frame_id] = std::get<ChBodyTransform>(child_tf);
            child_id = child_frame_id;
            child_to_parent = parent_body->GetFrameRefToAbs().GetInverse() * child_body->GetFrameRefToAbs();
        } else {
            const auto& [child_frame, child_frame_id] = std::get<ChFrameTransform>(child_tf);
            child_id = child_frame_id;
            child_to_parent = child_frame;
        }

        // Pack into IPC struct (use struct from ChROSIPCMessage.h)
        ipc::TFTransform tf_data;
        std::strncpy(tf_data.parent_frame, parent_id.c_str(), 63);
        tf_data.parent_frame[63] = '\0';
        std::strncpy(tf_data.child_frame, child_id.c_str(), 63);
        tf_data.child_frame[63] = '\0';
        
        auto pos = child_to_parent.GetPos();
        tf_data.pos_x = pos.x();
        tf_data.pos_y = pos.y();
        tf_data.pos_z = pos.z();
        
        auto quat = child_to_parent.GetRot();
        tf_data.rot_w = quat.e0();
        tf_data.rot_x = quat.e1();
        tf_data.rot_y = quat.e2();
        tf_data.rot_z = quat.e3();
        
        transform_data.push_back(tf_data);
    }
    
    // Serialize: header + transform array
    size_t total_size = sizeof(ipc::TFData) + transform_data.size() * sizeof(ipc::TFTransform);
    std::vector<uint8_t> bytes(total_size);
    
    ipc::TFData header;
    header.transform_count = static_cast<uint32_t>(transform_data.size());
    
    std::memcpy(bytes.data(), &header, sizeof(ipc::TFData));
    if (!transform_data.empty()) {
        std::memcpy(bytes.data() + sizeof(ipc::TFData), 
                    transform_data.data(), 
                    transform_data.size() * sizeof(ipc::TFTransform));
    }
    
    return bytes;
}

void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent,
                                  const std::string& parent_frame_id,
                                  std::shared_ptr<chrono::ChBody> child,
                                  const std::string& child_frame_id) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: Parent body is null" << std::endl;
        return;
    } else if (!child) {
        std::cerr << "ChROSTFHandler::AddTransform: Child body is null" << std::endl;
        return;
    }

    ChBodyTransform parent_tf = std::make_pair(parent, parent_frame_id.empty() ? parent->GetName() : parent_frame_id);
    ChBodyTransform child_tf = std::make_pair(child, child_frame_id.empty() ? child->GetName() : child_frame_id);
    m_transforms.push_back(std::make_pair(parent_tf, child_tf));
}

void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent,
                                  const std::string& parent_frame_id,
                                  chrono::ChFrame<double> child_frame,
                                  const std::string& child_frame_id) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: Parent body is null" << std::endl;
        return;
    }

    ChBodyTransform parent_tf = std::make_pair(parent, parent_frame_id);
    ChFrameTransform child_tf = std::make_pair(child_frame, child_frame_id);
    m_transforms.push_back(std::make_pair(parent_tf, child_tf));
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
        AddTransform(parent, joint->parent_link_name, child, joint->child_link_name);
    }
}
#endif

#ifdef CHRONO_SENSOR
void ChROSTFHandler::AddSensor(std::shared_ptr<chrono::sensor::ChSensor> sensor,
                               const std::string& parent_frame_id,
                               const std::string& child_frame_id) {
    if (!sensor) {
        std::cerr << "ChROSTFHandler::AddSensor: Sensor is null" << std::endl;
        return;
    }

    auto parent = sensor->GetParent();
    auto child_frame = sensor->GetOffsetPose();
    AddTransform(parent, parent_frame_id, child_frame, child_frame_id);
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

}  // namespace ros
}  // namespace chrono
