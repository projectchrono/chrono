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
// ROS Handler for communicating gyroscope information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"

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
    : ChROSHandler(update_rate), m_imu(imu), m_topic_name(topic_name) {}

bool ChROSGyroscopeHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterGyroAccess, ChFilterGyroAccessName>(m_imu)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::Imu>(m_topic_name, 1);

    m_imu_msg.header.frame_id = m_imu->GetName();

    return true;
}

void ChROSGyroscopeHandler::Tick(double time) {
    auto imu_ptr = m_imu->GetMostRecentBuffer<UserGyroBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        std::cout << "Gyroscope buffer is not ready. Not ticking." << std::endl;
        return;
    }

    GyroData imu_data = imu_ptr->Buffer[0];
    m_imu_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_imu_msg.angular_velocity.x = imu_data.Roll;
    m_imu_msg.angular_velocity.y = imu_data.Pitch;
    m_imu_msg.angular_velocity.z = imu_data.Yaw;

    // Update the covariance matrix
    // The ChGyroscopeSensor does not currently support covariances, so we'll
    // use the imu message to store a rolling average of the covariance
    m_imu_msg.angular_velocity_covariance = CalculateCovariance(imu_data);

    m_publisher->publish(m_imu_msg);
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