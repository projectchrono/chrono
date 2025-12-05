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
// ROS Handler for communicating imu information. Packages the data from other
// imu handlers into a single message and publishes it.
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"
#include "chrono_ros/handlers/sensor/ChROSIMUHandler_ipc.h"

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

std::vector<uint8_t> ChROSIMUHandler::GetSerializedData(double time) {
    if (!m_accel_handler || !m_gyro_handler || !m_mag_handler) {
        // std::cerr << "IMU handler not properly initialized. Not ticking." << std::endl;
        return {};
    }

    ipc::IMUData msg;
    strncpy(msg.topic_name, m_topic_name.c_str(), sizeof(msg.topic_name) - 1);
    strncpy(msg.frame_id, m_frame_id.c_str(), sizeof(msg.frame_id) - 1);
    
    msg.has_accel = false;
    msg.has_gyro = false;
    msg.has_mag = false;

    if (m_accel_handler) {
        // Ensure the handler has updated data for this time step
        m_accel_handler->GetSerializedData(time);
        const auto& accel_data = m_accel_handler->m_last_data_struct;
        
        msg.has_accel = true;
        std::memcpy(msg.linear_acceleration, accel_data.linear_acceleration, sizeof(msg.linear_acceleration));
        std::memcpy(msg.linear_acceleration_covariance, accel_data.linear_acceleration_covariance, sizeof(msg.linear_acceleration_covariance));
    }
    
    if (m_gyro_handler) {
        m_gyro_handler->GetSerializedData(time);
        const auto& gyro_data = m_gyro_handler->m_last_data_struct;
        
        msg.has_gyro = true;
        std::memcpy(msg.angular_velocity, gyro_data.angular_velocity, sizeof(msg.angular_velocity));
        std::memcpy(msg.angular_velocity_covariance, gyro_data.angular_velocity_covariance, sizeof(msg.angular_velocity_covariance));
    }
    
    if (m_mag_handler) {
        m_mag_handler->GetSerializedData(time);
        const auto& mag_data = m_mag_handler->m_last_data_struct;
        
        msg.has_mag = true;
        // Placeholder for future implementation
    }

    std::vector<uint8_t> buffer(sizeof(ipc::IMUData));
    std::memcpy(buffer.data(), &msg, sizeof(ipc::IMUData));

    return buffer;
}

}  // namespace ros
}  // namespace chrono