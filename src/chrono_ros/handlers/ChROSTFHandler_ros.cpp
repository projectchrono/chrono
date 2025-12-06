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
// ROS publishing implementation for TFHandler
// This file is ONLY compiled and linked in the subprocess (chrono_ros_node)
//
// PUBLISHER PATTERN (variable-size data):
// Receives array of transforms from main process and publishes to /tf topic
// Demonstrates handling variable-size IPC messages
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <cstring>
#include <vector>

namespace chrono {
namespace ros {

/// ROS publishing function for TF handler
/// Receives variable-size array of transforms and publishes to /tf
void PublishTFToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    // Channel not needed for publishers
    (void)channel;
    
    // Deserialize header
    if (data_size < sizeof(ipc::TFData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid TF data size: %zu, expected at least %zu", 
                     data_size, sizeof(ipc::TFData));
        return;
    }
    
    ipc::TFData header;
    std::memcpy(&header, data, sizeof(ipc::TFData));
    
    // Validate data size
    size_t expected_size = sizeof(ipc::TFData) + header.transform_count * sizeof(ipc::TFTransform);
    if (data_size != expected_size) {
        RCLCPP_ERROR(node->get_logger(), "TF data size mismatch: got %zu, expected %zu for %u transforms",
                     data_size, expected_size, header.transform_count);
        return;
    }
    
    // Create broadcaster if needed (lazy initialization)
    static std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    if (!tf_broadcaster) {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
        RCLCPP_INFO(node->get_logger(), "Created TF broadcaster on /tf");
    }
    
    // Extract transforms
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(header.transform_count);
    
    const uint8_t* transform_data_ptr = data + sizeof(ipc::TFData);
    for (uint32_t i = 0; i < header.transform_count; ++i) {
        ipc::TFTransform tf_data;
        std::memcpy(&tf_data, transform_data_ptr + i * sizeof(ipc::TFTransform), 
                    sizeof(ipc::TFTransform));
        
        // Create ROS message
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = node->now();
        tf_msg.header.frame_id = std::string(tf_data.parent_frame);
        tf_msg.child_frame_id = std::string(tf_data.child_frame);
        
        tf_msg.transform.translation.x = tf_data.pos_x;
        tf_msg.transform.translation.y = tf_data.pos_y;
        tf_msg.transform.translation.z = tf_data.pos_z;
        
        tf_msg.transform.rotation.w = tf_data.rot_w;
        tf_msg.transform.rotation.x = tf_data.rot_x;
        tf_msg.transform.rotation.y = tf_data.rot_y;
        tf_msg.transform.rotation.z = tf_data.rot_z;
        
        transforms.push_back(tf_msg);
    }
    
    // Publish all transforms
    if (!transforms.empty()) {
        tf_broadcaster->sendTransform(transforms);
    }
}

// Register this handler's publish function with the registry
CHRONO_ROS_REGISTER_HANDLER(TF_DATA, PublishTFToROS)

}  // namespace ros
}  // namespace chrono
