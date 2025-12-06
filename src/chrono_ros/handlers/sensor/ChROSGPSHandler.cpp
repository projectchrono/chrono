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
// ROS Handler for communicating gps information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler_ipc.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/utils/ChGPSUtils.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGPSHandler::ChROSGPSHandler(std::shared_ptr<ChGPSSensor> gps, const std::string& topic_name)
    : ChROSGPSHandler(gps->GetUpdateRate(), gps, topic_name) {}

ChROSGPSHandler::ChROSGPSHandler(double update_rate, std::shared_ptr<ChGPSSensor> gps, const std::string& topic_name)
    : ChROSHandler(update_rate), m_gps(gps), m_topic_name(topic_name), m_running_average({0, 0, 0}) {}

bool ChROSGPSHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterGPSAccess, ChFilterGPSAccessName>(m_gps)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    return true;
}

std::vector<uint8_t> ChROSGPSHandler::GetSerializedData(double time) {
    // if (!ShouldTick(time)) {
    //     return {};
    // }

    auto gps_ptr = m_gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (!gps_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        // std::cout << "GPS buffer is not ready. Not ticking." << std::endl;
        return {};
    }

    GPSData gps_data = gps_ptr->Buffer[0];
    
    ipc::GPSData msg;
    strncpy(msg.topic_name, m_topic_name.c_str(), sizeof(msg.topic_name) - 1);
    strncpy(msg.frame_id, m_gps->GetName().c_str(), sizeof(msg.frame_id) - 1);
    msg.time = gps_data.Time;
    msg.latitude = gps_data.Latitude;
    msg.longitude = gps_data.Longitude;
    msg.altitude = gps_data.Altitude;

    // Update the covariance matrix
    auto covariance = CalculateCovariance(gps_data);
    IncrementTickCount();
    std::memcpy(msg.position_covariance, covariance.data(), sizeof(msg.position_covariance));

    std::vector<uint8_t> buffer(sizeof(ipc::GPSData));
    std::memcpy(buffer.data(), &msg, sizeof(ipc::GPSData));

    return buffer;
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