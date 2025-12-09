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
// ROS publishing implementation for BodyHandler
// This file is ONLY compiled and linked in the subprocess (chrono_ros_node)
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

/// ROS publishing function for body handler
/// This is called in the subprocess when body data arrives via IPC
void PublishBodyToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    // Channel not needed for publishers (only for subscribers sending back)
    (void)channel;
    // Deserialize the data
    if (data_size != sizeof(ChROSBodyData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid body data size: %zu, expected %zu", 
                     data_size, sizeof(ChROSBodyData));
        return;
    }
    
    ChROSBodyData body_data;
    std::memcpy(&body_data, data, sizeof(ChROSBodyData));
    
    std::string body_key = std::string(body_data.body_name);
    
    // Static publishers map - maintains publishers for all bodies
    static std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers;
    static std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_publishers;
    static std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr> accel_publishers;
    
    // Create publishers if they don't exist for this body
    std::string pose_topic = std::string(body_data.topic_prefix) + "/pose";
    if (pose_publishers.find(body_key) == pose_publishers.end()) {
        pose_publishers[body_key] = node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 1);
        RCLCPP_INFO(node->get_logger(), "Created body pose publisher: %s", pose_topic.c_str());
    }
    
    std::string twist_topic = std::string(body_data.topic_prefix) + "/twist";
    if (twist_publishers.find(body_key) == twist_publishers.end()) {
        twist_publishers[body_key] = node->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, 1);
        RCLCPP_INFO(node->get_logger(), "Created body twist publisher: %s", twist_topic.c_str());
    }
    
    std::string accel_topic = std::string(body_data.topic_prefix) + "/accel";
    if (accel_publishers.find(body_key) == accel_publishers.end()) {
        accel_publishers[body_key] = node->create_publisher<geometry_msgs::msg::AccelStamped>(accel_topic, 1);
        RCLCPP_INFO(node->get_logger(), "Created body accel publisher: %s", accel_topic.c_str());
    }
    
    // Get current ROS time
    auto stamp = node->get_clock()->now();
    
    // Publish pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = std::string(body_data.body_name);
    pose_msg.pose.position.x = body_data.pos_x;
    pose_msg.pose.position.y = body_data.pos_y;
    pose_msg.pose.position.z = body_data.pos_z;
    pose_msg.pose.orientation.w = body_data.rot_w;
    pose_msg.pose.orientation.x = body_data.rot_x;
    pose_msg.pose.orientation.y = body_data.rot_y;
    pose_msg.pose.orientation.z = body_data.rot_z;
    pose_publishers[body_key]->publish(pose_msg);
    
    // Publish twist
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = stamp;
    twist_msg.header.frame_id = std::string(body_data.body_name);
    twist_msg.twist.linear.x = body_data.lin_vel_x;
    twist_msg.twist.linear.y = body_data.lin_vel_y;
    twist_msg.twist.linear.z = body_data.lin_vel_z;
    twist_msg.twist.angular.x = body_data.ang_vel_x;
    twist_msg.twist.angular.y = body_data.ang_vel_y;
    twist_msg.twist.angular.z = body_data.ang_vel_z;
    twist_publishers[body_key]->publish(twist_msg);
    
    // Publish accel
    geometry_msgs::msg::AccelStamped accel_msg;
    accel_msg.header.stamp = stamp;
    accel_msg.header.frame_id = std::string(body_data.body_name);
    accel_msg.accel.linear.x = body_data.lin_acc_x;
    accel_msg.accel.linear.y = body_data.lin_acc_y;
    accel_msg.accel.linear.z = body_data.lin_acc_z;
    accel_msg.accel.angular.x = body_data.ang_acc_x;
    accel_msg.accel.angular.y = body_data.ang_acc_y;
    accel_msg.accel.angular.z = body_data.ang_acc_z;
    accel_publishers[body_key]->publish(accel_msg);
}

// Register this handler's publish function with the registry
CHRONO_ROS_REGISTER_HANDLER(BODY_DATA, PublishBodyToROS)

}  // namespace ros
}  // namespace chrono