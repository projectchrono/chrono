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
// ROS Handler for communicating magnetometer information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler_ipc.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSMagnetometerHandler::ChROSMagnetometerHandler(std::shared_ptr<ChMagnetometerSensor> imu,
                                                   const std::string& topic_name)
    : ChROSMagnetometerHandler(imu->GetUpdateRate(), imu, topic_name) {}

ChROSMagnetometerHandler::ChROSMagnetometerHandler(double update_rate,
                                                   std::shared_ptr<ChMagnetometerSensor> imu,
                                                   const std::string& topic_name)
    : ChROSHandler(update_rate), m_imu(imu), m_topic_name(topic_name), m_running_average({0, 0, 0}) {}

bool ChROSMagnetometerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterMagnetAccess, ChFilterMagnetAccessName>(m_imu)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    return true;
}

std::vector<uint8_t> ChROSMagnetometerHandler::GetSerializedData(double time) {
    if (time == m_last_time) {
        return m_last_serialized_data;
    }

    // if (!ShouldTick(time)) {
    //     return {};
    // }

    auto imu_ptr = m_imu->GetMostRecentBuffer<UserMagnetBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        // std::cout << "Magnetometer buffer is not ready. Not ticking." << std::endl;
        return {};
    }

    MagnetData imu_data = imu_ptr->Buffer[0];
    
    ipc::MagnetometerData msg;
    strncpy(msg.topic_name, m_topic_name.c_str(), sizeof(msg.topic_name) - 1);
    strncpy(msg.frame_id, m_imu->GetName().c_str(), sizeof(msg.frame_id) - 1);
    
    msg.magnetic_field[0] = imu_data.X;
    msg.magnetic_field[1] = imu_data.Y;
    msg.magnetic_field[2] = imu_data.Z;

    // Update the covariance matrix
    auto covariance = CalculateCovariance(imu_data);
    IncrementTickCount();
    std::memcpy(msg.magnetic_field_covariance, covariance.data(), sizeof(msg.magnetic_field_covariance));

    std::vector<uint8_t> buffer(sizeof(ipc::MagnetometerData));
    std::memcpy(buffer.data(), &msg, sizeof(ipc::MagnetometerData));

    m_last_time = time;
    m_last_serialized_data = buffer;
    m_last_data_struct = msg;

    return buffer;
}

std::array<double, 9> ChROSMagnetometerHandler::CalculateCovariance(const MagnetData& imu_data) {
    std::array<double, 3> imu_data_array = {imu_data.X, imu_data.Y, imu_data.Z};

    // Update the running average
    for (int i = 0; i < 3; i++)
        m_running_average[i] += (imu_data_array[i] - m_running_average[i]) / (GetTickCount() + 1);

    // Calculate and return the covariance
    auto count = (GetTickCount() > 1 ? GetTickCount() - 1 : 1);  // Avoid divide by zero (if only one tick, count = 1)
    return ChROSSensorHandlerUtilities::CalculateCovariance(imu_data_array, m_running_average, count);
}

}  // namespace ros
}  // namespace chrono