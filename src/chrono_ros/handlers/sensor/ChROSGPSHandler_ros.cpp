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
// ROS publishing implementation for GPSHandler
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler_ipc.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

void PublishGPSToROS(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    (void)channel;
    if (data_size != sizeof(ipc::GPSData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid GPS data size: %zu, expected %zu", 
                     data_size, sizeof(ipc::GPSData));
        return;
    }

    ipc::GPSData msg_data;
    std::memcpy(&msg_data, data, sizeof(ipc::GPSData));

    std::string topic_name = std::string(msg_data.topic_name);

    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr> publishers;

    if (publishers.find(topic_name) == publishers.end()) {
        publishers[topic_name] = node->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name, 1);
        RCLCPP_INFO(node->get_logger(), "Created GPS publisher: %s", topic_name.c_str());
    }

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(msg_data.time);
    msg.header.frame_id = std::string(msg_data.frame_id);
    
    msg.latitude = msg_data.latitude;
    msg.longitude = msg_data.longitude;
    msg.altitude = msg_data.altitude;

    std::memcpy(msg.position_covariance.data(), msg_data.position_covariance, sizeof(msg_data.position_covariance));
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    publishers[topic_name]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(GPS_DATA, PublishGPSToROS)

}  // namespace ros
}  // namespace chrono
