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
// ROS publishing implementation for LidarHandler
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler_ipc.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cstring>
#include <unordered_map>
#include <string>

namespace chrono {
namespace ros {

void PublishLidarPointCloud(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    (void)channel;

    if (data_size < sizeof(ipc::LidarPointCloudData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid lidar point cloud data size");
        return;
    }

    ipc::LidarPointCloudData header;
    std::memcpy(&header, data, sizeof(ipc::LidarPointCloudData));

    size_t point_size = 16; // sizeof(float) * 4 (x, y, z, intensity)
    size_t expected_size = sizeof(ipc::LidarPointCloudData) + (header.width * header.height * point_size);
    if (data_size != expected_size) {
        RCLCPP_ERROR(node->get_logger(), "Lidar point cloud data size mismatch");
        return;
    }

    std::string topic_key = std::string(header.topic_name);
    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers;

    if (publishers.find(topic_key) == publishers.end()) {
        publishers[topic_key] = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_key, 1);
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = std::string(header.frame_id);
    msg.width = header.width;
    msg.height = header.height;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.row_step = point_size * msg.width;
    msg.point_step = point_size;
    
    msg.fields.resize(4);
    const std::string field_names[4] = {"x", "y", "z", "intensity"};
    for (int i = 0; i < 4; i++) {
        msg.fields[i].name = field_names[i];
        msg.fields[i].offset = sizeof(float) * i;
        msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[i].count = 1;
    }

    size_t data_len = header.width * header.height * point_size;
    msg.data.resize(data_len);
    std::memcpy(msg.data.data(), data + sizeof(ipc::LidarPointCloudData), data_len);

    publishers[topic_key]->publish(msg);
}

void PublishLidarLaserScan(const uint8_t* data, size_t data_size, rclcpp::Node::SharedPtr node, ipc::IPCChannel* channel) {
    (void)channel;

    if (data_size < sizeof(ipc::LidarLaserScanData)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid lidar laser scan data size");
        return;
    }

    ipc::LidarLaserScanData header;
    std::memcpy(&header, data, sizeof(ipc::LidarLaserScanData));

    size_t ranges_size = header.count * sizeof(float);
    size_t intensities_size = header.count * sizeof(float);
    size_t expected_size = sizeof(ipc::LidarLaserScanData) + ranges_size + intensities_size;

    if (data_size != expected_size) {
        RCLCPP_ERROR(node->get_logger(), "Lidar laser scan data size mismatch");
        return;
    }

    std::string topic_key = std::string(header.topic_name);
    static std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> publishers;

    if (publishers.find(topic_key) == publishers.end()) {
        publishers[topic_key] = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_key, 1);
    }

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = std::string(header.frame_id);
    msg.angle_min = header.angle_min;
    msg.angle_max = header.angle_max;
    msg.angle_increment = header.angle_increment;
    msg.time_increment = header.time_increment;
    msg.scan_time = header.scan_time;
    msg.range_min = header.range_min;
    msg.range_max = header.range_max;

    msg.ranges.resize(header.count);
    msg.intensities.resize(header.count);

    const float* ranges_ptr = reinterpret_cast<const float*>(data + sizeof(ipc::LidarLaserScanData));
    const float* intensities_ptr = ranges_ptr + header.count;

    std::memcpy(msg.ranges.data(), ranges_ptr, ranges_size);
    std::memcpy(msg.intensities.data(), intensities_ptr, intensities_size);

    publishers[topic_key]->publish(msg);
}

CHRONO_ROS_REGISTER_HANDLER(LIDAR_POINTCLOUD, PublishLidarPointCloud)
CHRONO_ROS_REGISTER_HANDLER(LIDAR_LASERSCAN, PublishLidarLaserScan)

}  // namespace ros
}  // namespace chrono
