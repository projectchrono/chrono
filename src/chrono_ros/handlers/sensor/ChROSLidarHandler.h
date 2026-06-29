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
// ROS handler for a ChLidarSensor (sensor_msgs/msg/PointCloud2 or LaserScan).
//
// =============================================================================

#ifndef CH_ROS_LIDAR_HANDLER_H
#define CH_ROS_LIDAR_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_sensor_handlers
/// @{

/// Output message type for ChROSLidarHandler.
enum class ChROSLidarHandlerMessageType { LASER_SCAN, POINT_CLOUD2 };

/// Publishes a ChLidarSensor as either sensor_msgs/msg/PointCloud2 (xyzi point
/// cloud, the default) or sensor_msgs/msg/LaserScan (a single ring of ranges +
/// intensities). PointCloud2 requires a ChFilterXYZIAccess filter; LaserScan
/// requires a ChFilterDIAccess filter.
class CH_ROS_API ChROSLidarHandler : public ChROSHandler {
  public:
    /// Tick at the sensor's own update rate.
    ChROSLidarHandler(std::shared_ptr<chrono::sensor::ChLidarSensor> lidar,
                      const std::string& topic_name,
                      ChROSLidarHandlerMessageType msg_type = ChROSLidarHandlerMessageType::POINT_CLOUD2);
    /// Tick at an explicit rate.
    ChROSLidarHandler(double update_rate,
                      std::shared_ptr<chrono::sensor::ChLidarSensor> lidar,
                      const std::string& topic_name,
                      ChROSLidarHandlerMessageType msg_type = ChROSLidarHandlerMessageType::POINT_CLOUD2);

    /// Creates the PointCloud2 or LaserScan publisher per the configured message type.
    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    /// Publishes the latest scan (skipped while no subscriber is connected).
    virtual void Tick(double time) override;

  private:
    bool InitPointCloud2(ChROSBridge& bridge);
    bool InitLaserScan(ChROSBridge& bridge);
    void TickPointCloud2(double time);
    void TickLaserScan(double time);

    std::shared_ptr<chrono::sensor::ChLidarSensor> m_lidar;
    const std::string m_topic_name;
    const ChROSLidarHandlerMessageType m_msg_type;
    std::shared_ptr<ChROSPublisher> m_publisher;

    unsigned int m_width = 0;
    unsigned int m_height = 0;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
