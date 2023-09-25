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
// ROS Handler for communicating accelerometer information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSAccelerometerHandler::ChROSAccelerometerHandler(std::shared_ptr<ChAccelerometerSensor> imu,
                                                     const std::string& topic_name)
    : ChROSAccelerometerHandler(imu->GetUpdateRate(), imu, topic_name) {}

ChROSAccelerometerHandler::ChROSAccelerometerHandler(double update_rate,
                                                     std::shared_ptr<ChAccelerometerSensor> imu,
                                                     const std::string& topic_name)
    : ChROSHandler(update_rate), m_imu(imu), m_topic_name(topic_name) {}

bool ChROSAccelerometerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterAccelAccess, ChFilterAccelAccessName>(m_imu)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::Imu>(m_topic_name, 1);

    // m_imu_msg.header.frame_id = ; // TODO

    return true;
}

void ChROSAccelerometerHandler::Tick(double time) {
    auto imu_ptr = m_imu->GetMostRecentBuffer<UserAccelBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Accelerometer buffer is not ready. Not ticking. \n";
        return;
    }

    AccelData imu_data = imu_ptr->Buffer[0];
    m_imu_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_imu_msg.linear_acceleration.x = imu_data.X;
    m_imu_msg.linear_acceleration.y = imu_data.Y;
    m_imu_msg.linear_acceleration.z = imu_data.Z;

    m_publisher->publish(m_imu_msg);
}

}  // namespace ros
}  // namespace chrono