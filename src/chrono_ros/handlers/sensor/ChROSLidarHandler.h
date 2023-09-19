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
    /// Constructor
    ChROSLidarHandler(std::shared_ptr<chrono::sensor::ChLidarSensor> lidar);

    /// Initializes the handler. Creates a publisher for point cloud data on the topic "~/output/lidar/lidar.GetName()/data".
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChLidarSensor> m_lidar;

    sensor_msgs::msg::PointCloud2 m_lidar_msg;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
