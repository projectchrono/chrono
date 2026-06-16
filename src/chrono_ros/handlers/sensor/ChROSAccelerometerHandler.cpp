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
// ROS handler for a ChAccelerometerSensor (publishes sensor_msgs/msg/Imu).
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

#include <iostream>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSAccelerometerHandler::ChROSAccelerometerHandler(std::shared_ptr<ChAccelerometerSensor> imu,
                                                     const std::string& topic_name)
    : ChROSAccelerometerHandler(imu->GetUpdateRate(), imu, topic_name) {}

ChROSAccelerometerHandler::ChROSAccelerometerHandler(double update_rate,
                                                     std::shared_ptr<ChAccelerometerSensor> imu,
                                                     const std::string& topic_name)
    : ChROSHandler(update_rate),
      m_imu(imu),
      m_topic_name(topic_name),
      m_running_average({0, 0, 0}),
      m_linear_acceleration_covariance({}) {}

bool ChROSAccelerometerHandler::Initialize(ChROSBridge& bridge) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterAccelAccess, ChFilterAccelAccessName>(m_imu)) {
        return false;
    }
    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/Imu");
    return true;
}

void ChROSAccelerometerHandler::Tick(double time) {
    auto buffer = m_imu->GetMostRecentBuffer<UserAccelBufferPtr>();
    if (!buffer->Buffer) {
        std::cout << "Accelerometer buffer not ready. Not ticking." << std::endl;
        return;
    }

    AccelData data = buffer->Buffer[0];
    m_linear_acceleration = chrono::ChVector3d(data.X, data.Y, data.Z);
    m_linear_acceleration_covariance = CalculateCovariance(data);

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_imu->GetName());
    msg.SetTime("header.stamp", time);
    msg.SetDouble("linear_acceleration.x", data.X);
    msg.SetDouble("linear_acceleration.y", data.Y);
    msg.SetDouble("linear_acceleration.z", data.Z);
    msg.SetBlobCopy("linear_acceleration_covariance", m_linear_acceleration_covariance.data(), 9);
    m_publisher->Publish(msg);
}

std::array<double, 9> ChROSAccelerometerHandler::CalculateCovariance(const AccelData& data) {
    std::array<double, 3> sample = {data.X, data.Y, data.Z};
    for (int i = 0; i < 3; i++)
        m_running_average[i] += (sample[i] - m_running_average[i]) / (GetTickCount() + 1);
    auto count = (GetTickCount() > 1 ? GetTickCount() - 1 : 1);  // avoid divide-by-zero on the first tick
    return ChROSSensorHandlerUtilities::CalculateCovariance(sample, m_running_average, count);
}

}  // namespace ros
}  // namespace chrono
