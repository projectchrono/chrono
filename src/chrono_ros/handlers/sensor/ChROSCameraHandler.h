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
// ROS Handler for communicating camera information via IPC
//
// =============================================================================

#ifndef CH_ROS_CAMERA_HANDLER
#define CH_ROS_CAMERA_HANDLER

#include "chrono_ros/ChROSHandler.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// Handler for publishing camera images to ROS via IPC communication.
///
/// PUBLISHER PATTERN (Chrono → ROS):
/// - Main process: Extracts RGBA8 image data from ChCameraSensor
/// - Main process: Serializes metadata + pixel data into byte vector
/// - Subprocess: Deserializes and publishes as sensor_msgs::msg::Image
///
/// Data flow:
/// ChCameraSensor → Main process → IPC → Subprocess → ROS topic
///
/// Implementation files:
/// - ChROSCameraHandler.cpp: Main process logic (extract and serialize)
/// - ChROSCameraHandler_ros.cpp: Subprocess ROS publisher
/// - ChROSCameraHandler_ipc.h: IPC data structure definition
///
/// To implement a similar sensor publisher:
/// 1. Create IPC struct in _ipc.h with sensor metadata (plain C++ types)
/// 2. In handler.cpp: Extract sensor data in GetSerializedData()
/// 3. Serialize metadata + raw data to byte vector
/// 4. In _ros.cpp: Deserialize, create ROS message, publish
/// 5. Register with CHRONO_ROS_REGISTER_HANDLER macro
/// 6. Add message type to ChROSIPCMessage.h enum
/// 7. Add handler recognition to ChROSManager::GetHandlerMessageType()
class CH_ROS_API ChROSCameraHandler : public ChROSHandler {
  public:
    /// Constructor with automatic update rate from sensor
    /// @param camera Camera sensor to publish data from
    /// @param topic_name ROS topic to publish images to
    ChROSCameraHandler(std::shared_ptr<chrono::sensor::ChCameraSensor> camera, const std::string& topic_name);

    /// Constructor with custom update rate
    /// @param update_rate Rate at which to publish images (Hz)
    /// @param camera Camera sensor to publish data from
    /// @param topic_name ROS topic to publish images to
    ChROSCameraHandler(double update_rate,
                       std::shared_ptr<chrono::sensor::ChCameraSensor> camera,
                       const std::string& topic_name);

    /// Initialize handler (called once at startup in main process)
    /// In IPC mode, validates sensor has required filter, but doesn't create ROS publisher
    /// Subprocess will create the actual ROS publisher
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    
    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::CAMERA_DATA; }

    /// Extract and serialize camera image data for IPC transmission
    /// Called in main process at update_rate frequency
    /// Returns metadata + pixel data as byte vector
    /// @param time Current simulation time
    /// @return Serialized camera data (header + RGBA8 pixels)
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChCameraSensor> m_camera;  ///< Camera sensor to read from
    const std::string m_topic_name;  ///< ROS topic name for publishing
    double m_last_publish_time;  ///< Last time this camera published (for throttling)
    std::vector<uint8_t> m_serialize_buffer; ///< Buffer for serialization to avoid reallocations
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
