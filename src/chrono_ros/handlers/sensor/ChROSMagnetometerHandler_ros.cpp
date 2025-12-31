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
// ROS publishing implementation for MagnetometerHandler
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler_ipc.h"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

void PublishMagnetometerToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    (void)channel;
    if (data_size != sizeof(ipc::MagnetometerData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid magnetometer data size: %zu, expected %zu", 
                     data_size, sizeof(ipc::MagnetometerData));
        return;
    }

    ipc::MagnetometerData msg_data;
    std::memcpy(&msg_data, data, sizeof(ipc::MagnetometerData));

    std::string topic_name = std::string(msg_data.topic_name);

    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr> publishers;

    if (publishers.find(topic_name) == publishers.end()) {
        publishers[topic_name] = node->create_publisher<sensor_msgs::msg::MagneticField>(topic_name, 1);
        RCLCPP_INFO(node->get_logger(), "Created magnetometer publisher: %s", topic_name.c_str());
    }

    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = std::string(msg_data.frame_id);
    
    msg.magnetic_field.x = msg_data.magnetic_field[0];
    msg.magnetic_field.y = msg_data.magnetic_field[1];
    msg.magnetic_field.z = msg_data.magnetic_field[2];

    std::memcpy(msg.magnetic_field_covariance.data(), msg_data.magnetic_field_covariance, sizeof(msg_data.magnetic_field_covariance));

    publishers[topic_name]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(MAGNETOMETER_DATA, PublishMagnetometerToROS)

}  // namespace ros
}  // namespace chrono
