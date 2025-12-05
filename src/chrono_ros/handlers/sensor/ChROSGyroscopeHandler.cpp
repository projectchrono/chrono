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
// ROS Handler for communicating gyroscope information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler_ipc.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGyroscopeHandler::ChROSGyroscopeHandler(std::shared_ptr<ChGyroscopeSensor> imu, const std::string& topic_name)
    : ChROSGyroscopeHandler(imu->GetUpdateRate(), imu, topic_name) {}

ChROSGyroscopeHandler::ChROSGyroscopeHandler(double update_rate,
                                             std::shared_ptr<ChGyroscopeSensor> imu,
                                             const std::string& topic_name)
    : ChROSHandler(update_rate), m_imu(imu), m_topic_name(topic_name), m_running_average({0, 0, 0}) {}

bool ChROSGyroscopeHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterGyroAccess, ChFilterGyroAccessName>(m_imu)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    return true;
}

std::vector<uint8_t> ChROSGyroscopeHandler::GetSerializedData(double time) {
    if (time == m_last_time) {
        return m_last_serialized_data;
    }

    // if (!ShouldTick(time)) {
    //     return {};
    // }

    auto imu_ptr = m_imu->GetMostRecentBuffer<UserGyroBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        // std::cout << "Gyroscope buffer is not ready. Not ticking." << std::endl;
        return {};
    }

    GyroData imu_data = imu_ptr->Buffer[0];
    
    ipc::GyroscopeData msg;
    strncpy(msg.topic_name, m_topic_name.c_str(), sizeof(msg.topic_name) - 1);
    strncpy(msg.frame_id, m_imu->GetName().c_str(), sizeof(msg.frame_id) - 1);
    
    msg.angular_velocity[0] = imu_data.Roll;
    msg.angular_velocity[1] = imu_data.Pitch;
    msg.angular_velocity[2] = imu_data.Yaw;

    // Update the covariance matrix
    auto covariance = CalculateCovariance(imu_data);
    IncrementTickCount();
    std::memcpy(msg.angular_velocity_covariance, covariance.data(), sizeof(msg.angular_velocity_covariance));

    std::vector<uint8_t> buffer(sizeof(ipc::GyroscopeData));
    std::memcpy(buffer.data(), &msg, sizeof(ipc::GyroscopeData));

    m_last_time = time;
    m_last_serialized_data = buffer;
    m_last_data_struct = msg;

    return buffer;
}

std::array<double, 9> ChROSGyroscopeHandler::CalculateCovariance(const GyroData& imu_data) {
    std::array<double, 3> imu_data_array = {imu_data.Roll, imu_data.Pitch, imu_data.Yaw};

    // Update the running average
    for (int i = 0; i < 3; i++) 
        m_running_average[i] += (imu_data_array[i] - m_running_average[i]) / (GetTickCount() + 1);

    // Calculate and return the covariance
    auto count = (GetTickCount() > 1 ? GetTickCount() - 1 : 1);  // Avoid divide by zero (if only one tick, count = 1)
    return ChROSSensorHandlerUtilities::CalculateCovariance(imu_data_array, m_running_average, count);
}

}  // namespace ros
}  // namespace chrono