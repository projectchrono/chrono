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
// ROS handler for a ChMagnetometerSensor (publishes sensor_msgs/msg/MagneticField).
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

#include <iostream>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSMagnetometerHandler::ChROSMagnetometerHandler(std::shared_ptr<ChMagnetometerSensor> imu,
                                                   const std::string& topic_name)
    : ChROSMagnetometerHandler(imu->GetUpdateRate(), imu, topic_name) {}

ChROSMagnetometerHandler::ChROSMagnetometerHandler(double update_rate,
                                                   std::shared_ptr<ChMagnetometerSensor> imu,
                                                   const std::string& topic_name)
    : ChROSHandler(update_rate),
      m_imu(imu),
      m_topic_name(topic_name),
      m_running_average({0, 0, 0}),
      m_magnetic_field_covariance({}) {}

bool ChROSMagnetometerHandler::Initialize(ChROSBridge& bridge) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterMagnetAccess, ChFilterMagnetAccessName>(m_imu)) {
        return false;
    }
    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/MagneticField");
    return true;
}

void ChROSMagnetometerHandler::Tick(double time) {
    auto buffer = m_imu->GetMostRecentBuffer<UserMagnetBufferPtr>();
    if (!buffer->Buffer) {
        std::cout << "Magnetometer: waiting for first sample..." << std::endl;  // normal during warm-up
        return;
    }

    MagnetData data = buffer->Buffer[0];
    m_magnetic_field = chrono::ChVector3d(data.X, data.Y, data.Z);
    m_magnetic_field_covariance = CalculateCovariance(data);

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_imu->GetName());
    msg.SetTime("header.stamp", time);
    msg.SetDouble("magnetic_field.x", data.X);
    msg.SetDouble("magnetic_field.y", data.Y);
    msg.SetDouble("magnetic_field.z", data.Z);
    msg.SetBlobCopy("magnetic_field_covariance", m_magnetic_field_covariance.data(), 9);
    m_publisher->Publish(msg);
}

std::array<double, 9> ChROSMagnetometerHandler::CalculateCovariance(const MagnetData& data) {
    std::array<double, 3> sample = {data.X, data.Y, data.Z};
    for (int i = 0; i < 3; i++)
        m_running_average[i] += (sample[i] - m_running_average[i]) / (GetTickCount() + 1);
    auto count = (GetTickCount() > 1 ? GetTickCount() - 1 : 1);
    return ChROSSensorHandlerUtilities::CalculateCovariance(sample, m_running_average, count);
}

}  // namespace ros
}  // namespace chrono
