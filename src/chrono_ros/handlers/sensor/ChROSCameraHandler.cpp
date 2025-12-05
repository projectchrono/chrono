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
// ROS Handler for communicating camera information
//
// =============================================================================

#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler_ipc.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include <cstring>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSCameraHandler::ChROSCameraHandler(std::shared_ptr<ChCameraSensor> camera, const std::string& topic_name)
    : ChROSCameraHandler(camera->GetUpdateRate(), camera, topic_name) {}

ChROSCameraHandler::ChROSCameraHandler(double update_rate,
                                       std::shared_ptr<ChCameraSensor> camera,
                                       const std::string& topic_name)
    : ChROSHandler(update_rate), m_camera(camera), m_topic_name(topic_name), m_last_publish_time(-1.0) {}

bool ChROSCameraHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Validate sensor has the required RGBA8 access filter
    if (!ChROSSensorHandlerUtilities::CheckSensorHasFilter<ChFilterRGBA8Access, ChFilterRGBA8AccessName>(m_camera)) {
        return false;
    }

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    // In IPC mode, no ROS publisher created here - subprocess will create it
    return true;
}

std::vector<uint8_t> ChROSCameraHandler::GetSerializedData(double time) {
    // Check if it's time to publish based on update rate
    // Handler base class doesn't automatically throttle GetSerializedData calls
    double frame_time = GetUpdateRate() == 0 ? 0 : 1.0 / GetUpdateRate();

    if (m_last_publish_time >= 0 && (time - m_last_publish_time) < frame_time) {
        // Not time yet - return empty to skip this frame
        return std::vector<uint8_t>();
    }
    
    // Extract camera image from Chrono sensor (no ROS symbols)
    auto rgba8_ptr = m_camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    if (!rgba8_ptr->Buffer) {
        // Buffer not ready - return empty to skip this frame
        return std::vector<uint8_t>();
    }

    // Calculate image dimensions
    uint32_t width = m_camera->GetWidth() / m_camera->GetSampleFactor();
    uint32_t height = m_camera->GetHeight() / m_camera->GetSampleFactor();
    uint32_t step = sizeof(PixelRGBA8) * width;
    size_t image_size = step * height;

    // Create IPC metadata header
    ipc::CameraData header;
    strncpy(header.topic_name, m_topic_name.c_str(), sizeof(header.topic_name) - 1);
    header.topic_name[sizeof(header.topic_name) - 1] = '\0';
    
    strncpy(header.frame_id, m_camera->GetName().c_str(), sizeof(header.frame_id) - 1);
    header.frame_id[sizeof(header.frame_id) - 1] = '\0';
    
    header.width = width;
    header.height = height;
    header.step = step;

    // Serialize: header + pixel data
    // Use member buffer to avoid reallocation
    size_t total_size = sizeof(ipc::CameraData) + image_size;
    if (m_serialize_buffer.capacity() < total_size) {
        m_serialize_buffer.reserve(total_size);
    }
    m_serialize_buffer.resize(total_size);
    
    // Copy header
    std::memcpy(m_serialize_buffer.data(), &header, sizeof(ipc::CameraData));
    
    // Copy pixel data
    const uint8_t* pixel_ptr = reinterpret_cast<const uint8_t*>(rgba8_ptr->Buffer.get());
    std::memcpy(m_serialize_buffer.data() + sizeof(ipc::CameraData), pixel_ptr, image_size);

    m_last_publish_time = time;
    return m_serialize_buffer;
}

}  // namespace ros
}  // namespace chrono
