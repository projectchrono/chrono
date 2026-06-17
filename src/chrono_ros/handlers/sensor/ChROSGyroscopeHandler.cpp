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
// ROS handler for a ChGyroscopeSensor (publishes sensor_msgs/msg/Imu).
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

#include <iostream>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGyroscopeHandler::ChROSGyroscopeHandler(std::shared_ptr<ChGyroscopeSensor> imu, const std::string& topic_name)
    : ChROSGyroscopeHandler(imu->GetUpdateRate(), imu, topic_name) {}

ChROSGyroscopeHandler::ChROSGyroscopeHandler(double update_rate,
                                             std::shared_ptr<ChGyroscopeSensor> imu,
                                             const std::string& topic_name)
    : ChROSHandler(update_rate),
      m_imu(imu),
      m_topic_name(topic_name),
      m_running_average({0, 0, 0}),
      m_angular_velocity_covariance({}) {}

bool ChROSGyroscopeHandler::Initialize(ChROSBridge& bridge) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterGyroAccess, ChFilterGyroAccessName>(m_imu)) {
        return false;
    }
    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/Imu");
    return true;
}

void ChROSGyroscopeHandler::Tick(double time) {
    auto buffer = m_imu->GetMostRecentBuffer<UserGyroBufferPtr>();
    if (!buffer->Buffer) {
        std::cout << "Gyroscope: waiting for first sample..." << std::endl;  // normal during warm-up
        return;
    }

    GyroData data = buffer->Buffer[0];
    m_angular_velocity = chrono::ChVector3d(data.Roll, data.Pitch, data.Yaw);
    m_angular_velocity_covariance = CalculateCovariance(data);

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_imu->GetName());
    msg.SetTime("header.stamp", time);
    msg.SetDouble("angular_velocity.x", data.Roll);
    msg.SetDouble("angular_velocity.y", data.Pitch);
    msg.SetDouble("angular_velocity.z", data.Yaw);
    msg.SetBlobCopy("angular_velocity_covariance", m_angular_velocity_covariance.data(), 9);
    m_publisher->Publish(msg);
}

std::array<double, 9> ChROSGyroscopeHandler::CalculateCovariance(const GyroData& data) {
    std::array<double, 3> sample = {data.Roll, data.Pitch, data.Yaw};
    for (int i = 0; i < 3; i++)
        m_running_average[i] += (sample[i] - m_running_average[i]) / (GetTickCount() + 1);
    auto count = (GetTickCount() > 1 ? GetTickCount() - 1 : 1);
    return ChROSSensorHandlerUtilities::CalculateCovariance(sample, m_running_average, count);
}

}  // namespace ros
}  // namespace chrono
