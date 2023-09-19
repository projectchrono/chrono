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

#ifndef CH_ROS_GPS_HANDLER
#define CH_ROS_GPS_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChGPSSensor.h"

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChGPSSensor to ROS. Will publish sensor_msgs::msg::NavSatFix.
class ChROSGPSHandler : public ChROSHandler {
  public:
    /// Constructor
    ChROSGPSHandler(std::shared_ptr<chrono::sensor::ChGPSSensor> gps);

    /// Initializes the handler. Creates a publisher for the gps data on the topic "~/output/gps/gps.GetName()/data"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChGPSSensor> m_gps;

    sensor_msgs::msg::NavSatFix m_gps_msg;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_publisher;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
