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
// ROS publishing implementation for IMUHandler
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/sensor/ChROSIMUHandler_ipc.h"
#include "sensor_msgs/msg/imu.hpp"

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

void PublishIMUToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    (void)channel;
    if (data_size != sizeof(ipc::IMUData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid IMU data size: %zu, expected %zu", 
                     data_size, sizeof(ipc::IMUData));
        return;
    }

    ipc::IMUData msg_data;
    std::memcpy(&msg_data, data, sizeof(ipc::IMUData));

    std::string topic_name = std::string(msg_data.topic_name);

    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> publishers;

    if (publishers.find(topic_name) == publishers.end()) {
        publishers[topic_name] = node->create_publisher<sensor_msgs::msg::Imu>(topic_name, 1);
        RCLCPP_INFO(node->get_logger(), "Created IMU publisher: %s", topic_name.c_str());
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = std::string(msg_data.frame_id);
    
    if (msg_data.has_accel) {
        msg.linear_acceleration.x = msg_data.linear_acceleration[0];
        msg.linear_acceleration.y = msg_data.linear_acceleration[1];
        msg.linear_acceleration.z = msg_data.linear_acceleration[2];
        std::memcpy(msg.linear_acceleration_covariance.data(), msg_data.linear_acceleration_covariance, sizeof(msg_data.linear_acceleration_covariance));
    }
    
    if (msg_data.has_gyro) {
        msg.angular_velocity.x = msg_data.angular_velocity[0];
        msg.angular_velocity.y = msg_data.angular_velocity[1];
        msg.angular_velocity.z = msg_data.angular_velocity[2];
        std::memcpy(msg.angular_velocity_covariance.data(), msg_data.angular_velocity_covariance, sizeof(msg_data.angular_velocity_covariance));
    }
    
    publishers[topic_name]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(IMU_DATA, PublishIMUToROS)

}  // namespace ros
}  // namespace chrono
