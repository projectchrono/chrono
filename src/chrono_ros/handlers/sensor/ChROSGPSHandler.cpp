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

    // m_gps_msg.header.frame_id = ; // TODO

    return true;
}

void ChROSGPSHandler::Tick(double time) {
    auto gps_ptr = m_gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (!gps_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "GPS buffer is not ready. Not ticking. \n";
        return;
    }

    GPSData gps_data = gps_ptr->Buffer[0];
    m_gps_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(gps_data.Time);
    m_gps_msg.latitude = gps_data.Latitude;
    m_gps_msg.longitude = gps_data.Longitude;
    m_gps_msg.altitude = gps_data.Altitude;

    m_publisher->publish(m_gps_msg);
}

}  // namespace ros
}  // namespace chrono