// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// ROS Handler for communicating lidar information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

class ChROSLidarHandlerImpl {
  public:
    ChROSLidarHandlerImpl(std::shared_ptr<ChLidarSensor> lidar, const std::string& topic_name)
        : lidar(lidar), topic_name(topic_name) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) = 0;

  protected:
    virtual void Tick(double time) = 0;

    const std::string topic_name;
    std::shared_ptr<ChLidarSensor> lidar;

    friend class ChROSLidarHandler;
};

class PointCloud2Impl : public ChROSLidarHandlerImpl {
  public:
    PointCloud2Impl(std::shared_ptr<ChLidarSensor> lidar, const std::string& topic_name)
        : ChROSLidarHandlerImpl(lidar, topic_name) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override {
        m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 1);

        m_msg.header.frame_id = lidar->GetName();
        m_msg.width = lidar->GetWidth();
        m_msg.height = lidar->GetHeight();
        m_msg.is_bigendian = false;
        m_msg.is_dense = true;
        m_msg.row_step = sizeof(PixelXYZI) * m_msg.width;
        m_msg.point_step = sizeof(PixelXYZI);
        m_msg.data.resize(m_msg.row_step * m_msg.height);

        m_msg.fields.resize(4);
        const std::string field_names[4] = {"x", "y", "z", "intensity"};
        for (int i = 0; i < 4; i++) {
            m_msg.fields[i].name = field_names[i];
            m_msg.fields[i].offset = sizeof(float) * i;
            m_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            m_msg.fields[i].count = 1;
        }

        return true;
    }

  private:
    virtual void Tick(double time) override {
        auto pc_ptr = lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
        if (!pc_ptr->Buffer) {
            // TODO: Is this supposed to happen?
            std::cout << "Lidar buffer is not ready. Not ticking." << std::endl;
            return;
        }

        m_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(pc_ptr->Buffer.get());
        m_msg.data.assign(ptr, ptr + m_msg.row_step * m_msg.height);

        m_publisher->publish(m_msg);
    }

    sensor_msgs::msg::PointCloud2 m_msg;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;
};

class LaserScanImpl : public ChROSLidarHandlerImpl {
  public:
    LaserScanImpl(std::shared_ptr<ChLidarSensor> lidar, const std::string& topic_name)
        : ChROSLidarHandlerImpl(lidar, topic_name) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override {
        if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterDIAccess, ChFilterDIAccessName>(lidar)) {
            return false;
        }

        m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 1);

        m_msg.header.frame_id = lidar->GetName();
        m_msg.angle_min = lidar->GetHFOV() / 2.0;
        m_msg.angle_max = lidar->GetHFOV() / 2.0;
        m_msg.angle_increment = lidar->GetHFOV() / lidar->GetWidth();
        m_msg.time_increment = 0.0;  // TODO
        m_msg.scan_time = 1.0 / lidar->GetUpdateRate();
        m_msg.range_min = 0.0;
        m_msg.range_max = lidar->GetMaxDistance();

        m_msg.ranges.resize(lidar->GetWidth());
        m_msg.intensities.resize(lidar->GetWidth());

        return true;
    }

  private:
    virtual void Tick(double time) override {
        auto pc_ptr = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
        if (!pc_ptr->Buffer) {
            // TODO: Is this supposed to happen?
            std::cout << "Lidar buffer is not ready. Not ticking." << std::endl;
            return;
        }

        m_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);

        auto begin = pc_ptr->Buffer.get();
        std::transform(begin, begin + lidar->GetWidth(), m_msg.intensities.begin(),
                       [](const PixelDI& p) { return p.intensity; });
        std::transform(begin, begin + lidar->GetWidth(), m_msg.ranges.begin(),
                       [](const PixelDI& p) { return p.range; });

        m_publisher->publish(m_msg);
    }

    sensor_msgs::msg::LaserScan m_msg;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_publisher;
};

ChROSLidarHandler::ChROSLidarHandler(std::shared_ptr<ChLidarSensor> lidar,
                                     const std::string& topic_name,
                                     ChROSLidarHandlerMessageType msg_type)
    : ChROSLidarHandler(lidar->GetUpdateRate(), lidar, topic_name, msg_type) {}

ChROSLidarHandler::ChROSLidarHandler(double update_rate,
                                     std::shared_ptr<ChLidarSensor> lidar,
                                     const std::string& topic_name,
                                     ChROSLidarHandlerMessageType msg_type)
    : ChROSHandler(update_rate) {
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

void ChROSLidarHandler::Tick(double time) {
    m_impl->Tick(time);
}

}  // namespace ros
}  // namespace chrono