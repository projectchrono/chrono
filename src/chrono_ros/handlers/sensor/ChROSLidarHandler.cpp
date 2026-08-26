// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// ROS handler for a ChLidarSensor (sensor_msgs/msg/PointCloud2 or LaserScan).
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSQoS.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

#include <cstdint>
#include <iostream>
#include <vector>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

// sensor_msgs/msg/PointField datatype constant (FLOAT32 = 7); the schema carries
// no symbolic constants.
static constexpr uint64_t POINTFIELD_FLOAT32 = 7;

ChROSLidarHandler::ChROSLidarHandler(std::shared_ptr<ChLidarSensor> lidar,
                                     const std::string& topic_name,
                                     ChROSLidarHandlerMessageType msg_type)
    : ChROSLidarHandler(lidar->GetUpdateRate(), lidar, topic_name, msg_type) {}

ChROSLidarHandler::ChROSLidarHandler(double update_rate,
                                     std::shared_ptr<ChLidarSensor> lidar,
                                     const std::string& topic_name,
                                     ChROSLidarHandlerMessageType msg_type)
    : ChROSHandler(update_rate), m_lidar(lidar), m_topic_name(topic_name), m_msg_type(msg_type) {}

bool ChROSLidarHandler::Initialize(ChROSBridge& bridge) {
    switch (m_msg_type) {
        case ChROSLidarHandlerMessageType::POINT_CLOUD2:
            return InitPointCloud2(bridge);
        case ChROSLidarHandlerMessageType::LASER_SCAN:
            return InitLaserScan(bridge);
    }
    return false;
}

void ChROSLidarHandler::Tick(double time) {
    // Skip extraction + transfer when no ROS subscriber is connected.
    if (m_publisher->GetSubscriptionCount() == 0)
        return;

    switch (m_msg_type) {
        case ChROSLidarHandlerMessageType::POINT_CLOUD2:
            TickPointCloud2(time);
            break;
        case ChROSLidarHandlerMessageType::LASER_SCAN:
            TickLaserScan(time);
            break;
    }
}

// --- PointCloud2 -------------------------------------------------------------

bool ChROSLidarHandler::InitPointCloud2(ChROSBridge& bridge) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterXYZIAccess, ChFilterXYZIAccessName>(m_lidar)) {
        return false;
    }
    m_width = m_lidar->GetWidth();
    m_height = m_lidar->GetHeight();
    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/PointCloud2", ChROSQoS::SensorData());
    return true;
}

void ChROSLidarHandler::TickPointCloud2(double time) {
    auto buffer = m_lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
    if (!buffer->Buffer) {
        std::cout << "Lidar: waiting for first scan..." << std::endl;  // normal during warm-up
        return;
    }

    const unsigned int point_step = static_cast<unsigned int>(sizeof(PixelXYZI));  // 16 (x,y,z,intensity)
    const unsigned int row_step = point_step * m_width;

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_lidar->GetName());
    msg.SetTime("header.stamp", time);
    msg.SetUInt("height", m_height);
    msg.SetUInt("width", m_width);
    msg.SetBool("is_bigendian", false);
    msg.SetBool("is_dense", true);
    msg.SetUInt("point_step", point_step);
    msg.SetUInt("row_step", row_step);

    // fields[]: x, y, z, intensity - each a FLOAT32 at successive 4-byte offsets.
    const char* field_names[4] = {"x", "y", "z", "intensity"};
    for (unsigned int i = 0; i < 4; i++) {
        auto field = msg.AppendMessage("fields");
        field.SetString("name", field_names[i]);
        field.SetUInt("offset", static_cast<uint64_t>(sizeof(float)) * i);
        field.SetUInt("datatype", POINTFIELD_FLOAT32);
        field.SetUInt("count", 1);
    }

    // data is uint8[], element count == byte count. Zero-copy (buffer alive
    // until Publish() returns in this Tick).
    msg.SetBlob("data", buffer->Buffer.get(), static_cast<size_t>(row_step) * m_height);
    m_publisher->Publish(msg);
}

// --- LaserScan ---------------------------------------------------------------

bool ChROSLidarHandler::InitLaserScan(ChROSBridge& bridge) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterDIAccess, ChFilterDIAccessName>(m_lidar)) {
        return false;
    }
    m_width = m_lidar->GetWidth();
    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/LaserScan", ChROSQoS::SensorData());
    return true;
}

void ChROSLidarHandler::TickLaserScan(double time) {
    auto buffer = m_lidar->GetMostRecentBuffer<UserDIBufferPtr>();
    if (!buffer->Buffer) {
        std::cout << "Lidar: waiting for first scan..." << std::endl;  // normal during warm-up
        return;
    }

    const float hfov = m_lidar->GetHFOV();

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_lidar->GetName());
    msg.SetTime("header.stamp", time);
    // Symmetric sweep about the sensor's forward axis: [-HFOV/2, +HFOV/2].
    msg.SetDouble("angle_min", -hfov / 2.0);
    msg.SetDouble("angle_max", hfov / 2.0);
    msg.SetDouble("angle_increment", m_width > 0 ? hfov / m_width : 0.0);
    msg.SetDouble("time_increment", 0.0);
    msg.SetDouble("scan_time", 1.0 / m_lidar->GetUpdateRate());
    msg.SetDouble("range_min", 0.0);
    msg.SetDouble("range_max", m_lidar->GetMaxDistance());

    // Split the interleaved depth/intensity buffer into the two float32 sequences.
    const PixelDI* di = buffer->Buffer.get();
    std::vector<float> ranges(m_width);
    std::vector<float> intensities(m_width);
    for (unsigned int i = 0; i < m_width; i++) {
        ranges[i] = di[i].range;
        intensities[i] = di[i].intensity;
    }
    msg.SetBlob("ranges", ranges.data(), m_width);
    msg.SetBlob("intensities", intensities.data(), m_width);
    m_publisher->Publish(msg);
}

}  // namespace ros
}  // namespace chrono
