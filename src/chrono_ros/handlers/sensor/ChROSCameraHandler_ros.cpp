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
// ROS publishing implementation for CameraHandler
// This file is ONLY compiled and linked in the subprocess (chrono_ros_node)
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler_ipc.h"
#include "sensor_msgs/msg/image.hpp"

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

/// ROS publishing function for camera handler
/// This is called in the subprocess when camera data arrives via IPC
void PublishCameraToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    // Channel not needed for publishers (only for subscribers sending back)
    (void)channel;
    
    // Validate minimum data size (header must be present)
    if (data_size < sizeof(ipc::CameraData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid camera data size: %zu, expected at least %zu", 
                     data_size, sizeof(ipc::CameraData));
        return;
    }
    
    // Deserialize header
    ipc::CameraData header;
    std::memcpy(&header, data, sizeof(ipc::CameraData));
    
    // Calculate expected image size
    size_t expected_size = sizeof(ipc::CameraData) + (header.step * header.height);
    if (data_size != expected_size) {
        RCLCPP_ERROR(node->get_logger(), "Camera data size mismatch: got %zu, expected %zu", 
                     data_size, expected_size);
        return;
    }
    
    std::string topic_key = std::string(header.topic_name);
    
    // Static publishers map - maintains publishers for all cameras
    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers;
    
    // Create publisher if it doesn't exist for this topic
    if (publishers.find(topic_key) == publishers.end()) {
        publishers[topic_key] = node->create_publisher<sensor_msgs::msg::Image>(topic_key, 1);
        RCLCPP_INFO(node->get_logger(), "Created camera publisher: %s (%ux%u)", 
                    topic_key.c_str(), header.width, header.height);
    }
    
    // Create and publish ROS image message
    // Use a static message to avoid reallocation if possible, but be careful with multiple cameras
    // For now, just optimize the vector resize
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = node->get_clock()->now();
    img_msg.header.frame_id = std::string(header.frame_id);
    img_msg.width = header.width;
    img_msg.height = header.height;
    img_msg.encoding = std::string(header.encoding);
    img_msg.step = header.step;
    
    // Copy pixel data (from just after header)
    size_t image_size = header.step * header.height;
    
    // Reserve to avoid multiple reallocations
    img_msg.data.resize(image_size);
    std::memcpy(img_msg.data.data(), data + sizeof(ipc::CameraData), image_size);
    
    publishers[topic_key]->publish(img_msg);
}

// Register camera handler with the dispatcher
CHRONO_ROS_REGISTER_HANDLER(CAMERA_DATA, PublishCameraToROS)

}  // namespace ros
}  // namespace chrono
