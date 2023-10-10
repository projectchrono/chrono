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

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

class ChROSLidarHandlerImpl;

/// This handler interfaces a ChLidarSensor to ROS. Will publish sensor_msgs::msg::PointCloud2.
class ChROSLidarHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to lidar->GetUpdateRate().
    ChROSLidarHandler(std::shared_ptr<chrono::sensor::ChLidarSensor> lidar,
                      const std::string& topic_name,
                      bool use_pc2 = true);

    /// Full constructor. Takes a ChLidarSensor, update rate, and topic name.
    ChROSLidarHandler(double update_rate,
                      std::shared_ptr<chrono::sensor::ChLidarSensor> lidar,
                      const std::string& topic_name,
                      bool use_pc2 = true);

    ~ChROSLidarHandler() = default;

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<ChROSLidarHandlerImpl> m_impl;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
