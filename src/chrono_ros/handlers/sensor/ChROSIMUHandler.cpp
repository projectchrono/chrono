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
// ROS handler that aggregates accelerometer + gyroscope (+ magnetometer) data
// into a single sensor_msgs/msg/Imu message.
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"

#include <cassert>
#include <iostream>

namespace chrono {
namespace ros {

ChROSIMUHandler::ChROSIMUHandler(double update_rate, const std::string& topic_name, const std::string& frame_id)
    : ChROSHandler(update_rate), m_topic_name(topic_name), m_frame_id(frame_id) {}

bool ChROSIMUHandler::Initialize(ChROSBridge& bridge) {
    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/Imu");
    return true;
}

void ChROSIMUHandler::SetAccelerometerHandler(std::shared_ptr<ChROSAccelerometerHandler> accel_handler) {
    assert(accel_handler->GetUpdateRate() >= GetUpdateRate());
    m_accel_handler = accel_handler;
}

void ChROSIMUHandler::SetGyroscopeHandler(std::shared_ptr<ChROSGyroscopeHandler> gyro_handler) {
    assert(gyro_handler->GetUpdateRate() >= GetUpdateRate());
    m_gyro_handler = gyro_handler;
}

void ChROSIMUHandler::SetMagnetometerHandler(std::shared_ptr<ChROSMagnetometerHandler> mag_handler) {
    assert(mag_handler->GetUpdateRate() >= GetUpdateRate());
    m_mag_handler = mag_handler;
}

void ChROSIMUHandler::Tick(double time) {
    // 9.0 required all three sub-handlers to be set before ticking. The magnetometer
    // is consumed only to preserve that contract - sensor_msgs/Imu has no magnetic
    // field, so its data is not packed (candidate to relax: make mag optional).
    if (!m_accel_handler || !m_gyro_handler || !m_mag_handler) {
        std::cerr << "IMU handler not fully configured (needs accelerometer, gyroscope, and "
                     "magnetometer sub-handlers). Not ticking."
                  << std::endl;
        return;
    }

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_frame_id);
    msg.SetTime("header.stamp", time);

    const auto& accel = m_accel_handler->m_linear_acceleration;
    msg.SetDouble("linear_acceleration.x", accel.x());
    msg.SetDouble("linear_acceleration.y", accel.y());
    msg.SetDouble("linear_acceleration.z", accel.z());
    msg.SetBlobCopy("linear_acceleration_covariance", m_accel_handler->m_linear_acceleration_covariance.data(), 9);

    const auto& gyro = m_gyro_handler->m_angular_velocity;
    msg.SetDouble("angular_velocity.x", gyro.x());
    msg.SetDouble("angular_velocity.y", gyro.y());
    msg.SetDouble("angular_velocity.z", gyro.z());
    msg.SetBlobCopy("angular_velocity_covariance", m_gyro_handler->m_angular_velocity_covariance.data(), 9);

    m_publisher->Publish(msg);
}

}  // namespace ros
}  // namespace chrono
