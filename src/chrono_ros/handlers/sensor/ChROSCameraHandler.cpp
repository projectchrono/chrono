// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// ROS handler for a ChCameraSensor (publishes sensor_msgs/msg/Image).
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"

#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSQoS.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

#include <iostream>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSCameraHandler::ChROSCameraHandler(std::shared_ptr<ChCameraSensor> camera, const std::string& topic_name)
    : ChROSCameraHandler(camera->GetUpdateRate(), camera, topic_name) {}

ChROSCameraHandler::ChROSCameraHandler(double update_rate,
                                       std::shared_ptr<ChCameraSensor> camera,
                                       const std::string& topic_name)
    : ChROSHandler(update_rate), m_camera(camera), m_topic_name(topic_name) {}

bool ChROSCameraHandler::Initialize(ChROSBridge& bridge) {
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterRGBA8Access, ChFilterRGBA8AccessName>(m_camera)) {
        return false;
    }

    m_width = m_camera->GetWidth() / m_camera->GetSampleFactor();
    m_height = m_camera->GetHeight() / m_camera->GetSampleFactor();
    m_step = static_cast<unsigned int>(sizeof(PixelRGBA8)) * m_width;

    m_publisher = bridge.CreatePublisher(m_topic_name, "sensor_msgs/msg/Image", ChROSQoS::SensorData());
    return true;
}

void ChROSCameraHandler::Tick(double time) {
    // Skip extraction + transfer entirely when no ROS subscriber is connected.
    if (m_publisher->GetSubscriptionCount() == 0)
        return;

    auto rgba8 = m_camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    if (!rgba8->Buffer) {
        std::cout << "Camera: waiting for first frame..." << std::endl;  // normal during warm-up
        return;
    }

    auto msg = m_publisher->NewMessage();
    msg.SetString("header.frame_id", m_camera->GetName());
    msg.SetTime("header.stamp", time);
    msg.SetUInt("height", m_height);
    msg.SetUInt("width", m_width);
    msg.SetString("encoding", "rgba8");
    msg.SetUInt("step", m_step);
    // data is uint8[], so element count == byte count. Zero-copy: the sensor's
    // buffer stays alive until Publish() returns within this same Tick.
    msg.SetBlob("data", rgba8->Buffer.get(), static_cast<size_t>(m_step) * m_height);
    m_publisher->Publish(msg);
}

}  // namespace ros
}  // namespace chrono
