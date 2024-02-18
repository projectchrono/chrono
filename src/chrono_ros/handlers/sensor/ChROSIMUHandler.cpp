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
// ROS Handler for communicating imu information. Packages the data from other
// imu handlers into a single message and publishes it.
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSIMUHandler::ChROSIMUHandler(double update_rate, const std::string& topic_name, const std::string& frame_id)
    : ChROSHandler(update_rate), m_topic_name(topic_name), m_frame_id(frame_id) {}

bool ChROSIMUHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::Imu>(m_topic_name, 1);

    m_imu_msg.header.frame_id = m_frame_id;

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
    if (!m_accel_handler || !m_gyro_handler || !m_mag_handler) {
        std::cerr << "IMU handler not properly initialized. Not ticking." << std::endl;
        return;
    }

    if (m_accel_handler) {
        m_imu_msg.linear_acceleration = m_accel_handler->m_imu_msg.linear_acceleration;
        m_imu_msg.linear_acceleration_covariance = m_accel_handler->m_imu_msg.linear_acceleration_covariance;
    }
    if (m_gyro_handler) {
        m_imu_msg.angular_velocity = m_gyro_handler->m_imu_msg.angular_velocity;
        m_imu_msg.angular_velocity_covariance = m_gyro_handler->m_imu_msg.angular_velocity_covariance;
    }
    if (m_mag_handler) {
        // Convert the magnetic field to orientation
        auto magnetic_field = m_mag_handler->m_mag_msg.magnetic_field;
        auto magnetic_field_cov = m_mag_handler->m_mag_msg.magnetic_field_covariance;
    }

    m_imu_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_publisher->publish(m_imu_msg);
}

}  // namespace ros
}  // namespace chrono