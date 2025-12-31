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
// ROS Handler for communicating lidar information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler_ipc.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include <cstring>
#include <algorithm>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

class ChROSLidarHandlerImpl {
  public:
    ChROSLidarHandlerImpl(std::shared_ptr<ChLidarSensor> lidar, const std::string& topic_name)
        : lidar(lidar), topic_name(topic_name) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) = 0;
    virtual std::vector<uint8_t> GetSerializedData(double time) = 0;

  protected:
    const std::string topic_name;
    std::shared_ptr<ChLidarSensor> lidar;
    std::vector<uint8_t> m_serialize_buffer;

    friend class ChROSLidarHandler;
};

class PointCloud2Impl : public ChROSLidarHandlerImpl {
  public:
    PointCloud2Impl(std::shared_ptr<ChLidarSensor> lidar, const std::string& topic_name)
        : ChROSLidarHandlerImpl(lidar, topic_name) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override {
        // In IPC mode, we just check if the sensor has the required filter
        // No publisher creation here - subprocess handles ROS
        return true;
    }

    virtual std::vector<uint8_t> GetSerializedData(double time) override {
        auto pc_ptr = lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
        if (!pc_ptr->Buffer) {
            return std::vector<uint8_t>();
        }

        uint32_t width = lidar->GetWidth();
        uint32_t height = lidar->GetHeight();
        size_t point_size = sizeof(PixelXYZI);
        size_t data_size = width * height * point_size;

        // Serialize: header + point data
        // Use member buffer to avoid reallocation
        size_t total_size = sizeof(ipc::LidarPointCloudData) + data_size;
        if (m_serialize_buffer.capacity() < total_size) {
            m_serialize_buffer.reserve(total_size);
        }
        m_serialize_buffer.resize(total_size);

        ipc::LidarPointCloudData* header = reinterpret_cast<ipc::LidarPointCloudData*>(m_serialize_buffer.data());
        strncpy(header->topic_name, topic_name.c_str(), sizeof(header->topic_name) - 1);
        header->topic_name[sizeof(header->topic_name) - 1] = '\0';
        
        strncpy(header->frame_id, lidar->GetName().c_str(), sizeof(header->frame_id) - 1);
        header->frame_id[sizeof(header->frame_id) - 1] = '\0';

        header->width = width;
        header->height = height;

        // Copy point data directly after header
        uint8_t* data_ptr = m_serialize_buffer.data() + sizeof(ipc::LidarPointCloudData);
        std::memcpy(data_ptr, pc_ptr->Buffer.get(), data_size);

        return m_serialize_buffer;
    }
};

class LaserScanImpl : public ChROSLidarHandlerImpl {
  public:
    LaserScanImpl(std::shared_ptr<ChLidarSensor> lidar, const std::string& topic_name)
        : ChROSLidarHandlerImpl(lidar, topic_name) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override {
        if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterDIAccess, ChFilterDIAccessName>(lidar)) {
            return false;
        }
        return true;
    }

    virtual std::vector<uint8_t> GetSerializedData(double time) override {
        auto pc_ptr = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
        if (!pc_ptr->Buffer) {
            return std::vector<uint8_t>();
        }

        uint32_t count = lidar->GetWidth();
        size_t ranges_size = count * sizeof(float);
        size_t intensities_size = count * sizeof(float);
        size_t total_size = sizeof(ipc::LidarLaserScanData) + ranges_size + intensities_size;

        if (m_serialize_buffer.capacity() < total_size) {
            m_serialize_buffer.reserve(total_size);
        }
        m_serialize_buffer.resize(total_size);

        ipc::LidarLaserScanData* header = reinterpret_cast<ipc::LidarLaserScanData*>(m_serialize_buffer.data());
        strncpy(header->topic_name, topic_name.c_str(), sizeof(header->topic_name) - 1);
        header->topic_name[sizeof(header->topic_name) - 1] = '\0';
        
        strncpy(header->frame_id, lidar->GetName().c_str(), sizeof(header->frame_id) - 1);
        header->frame_id[sizeof(header->frame_id) - 1] = '\0';

        // Assume lidar is centered for now
        header->angle_min = -lidar->GetHFOV() / 2.0;
        header->angle_max = lidar->GetHFOV() / 2.0;
        header->angle_increment = lidar->GetHFOV() / lidar->GetWidth();
        header->time_increment = 0.0;
        header->scan_time = 1.0 / lidar->GetUpdateRate();
        header->range_min = 0.0;
        header->range_max = lidar->GetMaxDistance();
        header->count = count;

        float* ranges_ptr = reinterpret_cast<float*>(m_serialize_buffer.data() + sizeof(ipc::LidarLaserScanData));
        float* intensities_ptr = ranges_ptr + count;

        auto begin = pc_ptr->Buffer.get();
        // Extract ranges and intensities
        for (uint32_t i = 0; i < count; ++i) {
            ranges_ptr[i] = begin[i].range;
            intensities_ptr[i] = begin[i].intensity;
        }

        return m_serialize_buffer;
    }
};

ChROSLidarHandler::ChROSLidarHandler(std::shared_ptr<ChLidarSensor> lidar,
                                     const std::string& topic_name,
                                     ChROSLidarHandlerMessageType msg_type)
    : ChROSLidarHandler(lidar->GetUpdateRate(), lidar, topic_name, msg_type) {}

ChROSLidarHandler::ChROSLidarHandler(double update_rate,
                                     std::shared_ptr<ChLidarSensor> lidar,
                                     const std::string& topic_name,
                                     ChROSLidarHandlerMessageType msg_type)
    : ChROSHandler(update_rate), m_type(msg_type) {
    switch (msg_type) {
        case ChROSLidarHandlerMessageType::POINT_CLOUD2:
            m_impl = std::make_shared<PointCloud2Impl>(lidar, topic_name);
            break;
        case ChROSLidarHandlerMessageType::LASER_SCAN:
            m_impl = std::make_shared<LaserScanImpl>(lidar, topic_name);
            break;
        default:
            throw std::runtime_error("Invalid ChROSLidarHandlerMessageType");
    }
}

bool ChROSLidarHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_impl->topic_name)) {
        return false;
    }

    return m_impl->Initialize(interface);
}

std::vector<uint8_t> ChROSLidarHandler::GetSerializedData(double time) {
    double frame_time = GetUpdateRate() == 0 ? 0 : 1.0 / GetUpdateRate();

    if (m_last_publish_time >= 0 && (time - m_last_publish_time) < frame_time) {
        return std::vector<uint8_t>();
    }

    auto data = m_impl->GetSerializedData(time);
    if (!data.empty()) {
        m_last_publish_time = time;
    }
    return data;
}

}  // namespace ros
}  // namespace chrono