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
// ROS Handler for communicating gps information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGPSHandler::ChROSGPSHandler(std::shared_ptr<ChGPSSensor> gps, const std::string& topic_name)
    : ChROSGPSHandler(gps->GetUpdateRate(), gps, topic_name) {}

ChROSGPSHandler::ChROSGPSHandler(double update_rate, std::shared_ptr<ChGPSSensor> gps, const std::string& topic_name)
    : ChROSHandler(update_rate), m_gps(gps), m_topic_name(topic_name) {}

bool ChROSGPSHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterGPSAccess, ChFilterGPSAccessName>(m_gps)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::NavSatFix>(m_topic_name, 1);

    m_gps_msg.header.frame_id = m_gps->GetName();

    return true;
}

void ChROSGPSHandler::Tick(double time) {
    auto gps_ptr = m_gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (!gps_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        std::cout << "GPS buffer is not ready. Not ticking." << std::endl;
        return;
    }

    GPSData gps_data = gps_ptr->Buffer[0];
    m_gps_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(gps_data.Time);
    m_gps_msg.latitude = gps_data.Latitude;
    m_gps_msg.longitude = gps_data.Longitude;
    m_gps_msg.altitude = gps_data.Altitude;

    // Update the covariance matrix
    // The ChGPSSensor does not currently support covariances, so we'll
    // use the imu message to store a rolling average of the covariance
    m_gps_msg.position_covariance = CalculateCovariance(gps_data);
    m_gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    m_publisher->publish(m_gps_msg);
}

std::array<double, 9> ChROSGPSHandler::CalculateCovariance(const GPSData& gps_data) {
    auto gps_coord = chrono::ChVector3d(gps_data.Latitude, gps_data.Longitude, gps_data.Altitude);
    auto gps_reference = m_gps->GetGPSReference();
    chrono::sensor::GPS2Cartesian(gps_coord, gps_reference);
    auto enu_data = gps_coord;

    std::array<double, 3> enu_data_array = {enu_data.x(), enu_data.y(), enu_data.z()};

    // Update the running average
    for (int i = 0; i < 3; i++) 
        m_running_average[i] += (enu_data_array[i] - m_running_average[i]) / (GetTickCount() + 1);

    // Calculate and return the covariance
    auto count = (GetTickCount() > 1 ? GetTickCount() - 1 : 1);  // Avoid divide by zero (if only one tick, count = 1)
    return ChROSSensorHandlerUtilities::CalculateCovariance(enu_data_array, m_running_average, count);
}

}  // namespace ros
}  // namespace chrono