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
// ROS Handler for communicating camera information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include <sstream>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSCameraHandler::ChROSCameraHandler(std::shared_ptr<ChCameraSensor> camera, const std::string& topic_name)
    : ChROSCameraHandler(camera->GetUpdateRate(), camera, topic_name) {}

ChROSCameraHandler::ChROSCameraHandler(double update_rate,
                                       std::shared_ptr<ChCameraSensor> camera,
                                       const std::string& topic_name)
    : ChROSHandler(update_rate), m_camera(camera), m_topic_name(topic_name) {}

bool ChROSCameraHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterRGBA8Access, ChFilterRGBA8AccessName>(m_camera)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::Image>(m_topic_name, 1);

    m_image.header.frame_id = m_camera->GetName();
    m_image.width = m_camera->GetWidth() / m_camera->GetSampleFactor();
    m_image.height = m_camera->GetHeight() / m_camera->GetSampleFactor();
    m_image.encoding = "rgba8";
    m_image.step = sizeof(PixelRGBA8) * m_image.width;
    m_image.data.resize(m_image.step * m_image.height);

    return true;
}

void ChROSCameraHandler::Tick(double time) {
    auto rgba8_ptr = m_camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    if (!rgba8_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        std::cout << "Camera buffer is not ready. Not ticking." << std::endl;
        return;
    }

    m_image.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(rgba8_ptr->Buffer.get());
    m_image.data.assign(ptr, ptr + m_image.step * m_image.height);

    // TODO: The buffer above may be released (?) after this call. Is this a problem? Guess is no.
    m_publisher->publish(m_image);
}

}  // namespace ros
}  // namespace chrono