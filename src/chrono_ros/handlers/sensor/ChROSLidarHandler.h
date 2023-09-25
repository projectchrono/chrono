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
// ROS Handler for communicating lidar information
//
// =============================================================================

#ifndef CH_ROS_LIDAR_HANDLER
#define CH_ROS_LIDAR_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler interfaces a ChLidarSensor to ROS. Will publish sensor_msgs::msg::PointCloud2.
class ChROSLidarHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to lidar->GetUpdateRate().
    ChROSLidarHandler(std::shared_ptr<chrono::sensor::ChLidarSensor> lidar, const std::string& topic_name);

    /// Full constructor. Takes a ChLidarSensor, update rate, and topic name.
    ChROSLidarHandler(double update_rate,
                      std::shared_ptr<chrono::sensor::ChLidarSensor> lidar,
                      const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChLidarSensor> m_lidar;  ///< handle to the lidar sensor

    const std::string m_topic_name;                                           ///< name of the topic to publish to
    sensor_msgs::msg::PointCloud2 m_lidar_msg;                                ///< message to publish
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;  ///< the publisher for the lidar message
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
