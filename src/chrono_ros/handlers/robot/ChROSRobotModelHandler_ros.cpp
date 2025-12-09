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
// ROS publishing implementation for RobotModelHandler
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/robot/ChROSRobotModelHandler_ipc.h"
#include "std_msgs/msg/string.hpp"

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

void PublishRobotModelToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    (void)channel;
    if (data_size < sizeof(ipc::RobotModelData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid robot model data size: %zu, expected at least %zu", 
                     data_size, sizeof(ipc::RobotModelData));
        return;
    }

    ipc::RobotModelData msg_data;
    std::memcpy(&msg_data, data, sizeof(ipc::RobotModelData));

    if (data_size < sizeof(ipc::RobotModelData) + msg_data.model_length) {
        RCLCPP_ERROR(node->get_logger(), "Invalid robot model data size: %zu, expected %zu", 
                     data_size, sizeof(ipc::RobotModelData) + msg_data.model_length);
        return;
    }

    std::string topic_name = std::string(msg_data.topic_name);
    std::string model_string(reinterpret_cast<const char*>(data + sizeof(ipc::RobotModelData)), msg_data.model_length);

    static std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers;

    if (publishers.find(topic_name) == publishers.end()) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        publishers[topic_name] = node->create_publisher<std_msgs::msg::String>(topic_name, qos);
        RCLCPP_INFO(node->get_logger(), "Created robot model publisher: %s", topic_name.c_str());
    }

    std_msgs::msg::String msg;
    msg.data = model_string;

    publishers[topic_name]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(ROBOT_MODEL_DATA, PublishRobotModelToROS)

}  // namespace ros
}  // namespace chrono
