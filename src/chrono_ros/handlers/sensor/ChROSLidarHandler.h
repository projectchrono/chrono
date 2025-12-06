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
// ROS Handler for communicating lidar information
//
// =============================================================================

#ifndef CH_ROS_LIDAR_HANDLER
#define CH_ROS_LIDAR_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

enum class ChROSLidarHandlerMessageType { LASER_SCAN, POINT_CLOUD2 };

class ChROSLidarHandlerImpl;

/// Handler for publishing Lidar data to ROS via IPC communication.
///
/// PUBLISHER PATTERN (Chrono → ROS):
/// - Main process: Extracts point cloud or laser scan data from ChLidarSensor
/// - Main process: Serializes metadata + raw data into byte vector
/// - Subprocess: Deserializes and publishes as sensor_msgs::msg::PointCloud2 or LaserScan
///
/// Data flow:
/// ChLidarSensor → Main process → IPC → Subprocess → ROS topic
///
/// Implementation files:
/// - ChROSLidarHandler.cpp: Main process logic (extract and serialize)
/// - ChROSLidarHandler_ros.cpp: Subprocess ROS publisher
/// - ChROSLidarHandler_ipc.h: IPC data structure definition
class CH_ROS_API ChROSLidarHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to lidar->GetUpdateRate().
    /// @param lidar Lidar sensor to publish data from
    /// @param topic_name ROS topic to publish data to
    /// @param msg_type Type of ROS message to publish (POINT_CLOUD2 or LASER_SCAN)
    ChROSLidarHandler(std::shared_ptr<chrono::sensor::ChLidarSensor> lidar,
                      const std::string& topic_name,
                      ChROSLidarHandlerMessageType msg_type = ChROSLidarHandlerMessageType::POINT_CLOUD2);

    /// Full constructor. Takes a ChLidarSensor, update rate, and topic name.
    /// @param update_rate Rate at which to publish data (Hz)
    /// @param lidar Lidar sensor to publish data from
    /// @param topic_name ROS topic to publish data to
    /// @param msg_type Type of ROS message to publish (POINT_CLOUD2 or LASER_SCAN)
    ChROSLidarHandler(double update_rate,
                      std::shared_ptr<chrono::sensor::ChLidarSensor> lidar,
                      const std::string& topic_name,
                      ChROSLidarHandlerMessageType msg_type = ChROSLidarHandlerMessageType::POINT_CLOUD2);

    /// Initializes the handler.
    /// In IPC mode, validates sensor has required filter, but doesn't create ROS publisher.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Extract and serialize lidar data for IPC transmission.
    /// Called in main process at update_rate frequency.
    /// Returns metadata + raw data as byte vector.
    /// @param time Current simulation time
    /// @return Serialized lidar data (header + data)
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override {
        return (m_type == ChROSLidarHandlerMessageType::POINT_CLOUD2) 
               ? ipc::MessageType::LIDAR_POINTCLOUD 
               : ipc::MessageType::LIDAR_LASERSCAN;
    }

  private:
    std::shared_ptr<ChROSLidarHandlerImpl> m_impl;
    double m_last_publish_time = -1.0;
    ChROSLidarHandlerMessageType m_type;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
