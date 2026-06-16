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
// ROS handler for a ChGPSSensor (publishes sensor_msgs/msg/NavSatFix).
//
// =============================================================================

#ifndef CH_ROS_GPS_HANDLER_H
#define CH_ROS_GPS_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChGPSSensor.h"

#include <array>
#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_sensor_handlers
/// @{

/// Publishes a ChGPSSensor as sensor_msgs/msg/NavSatFix. The position covariance
/// is approximated from a running mean in ENU space (the sensor emits none).
/// Constructors match 9.0.
class CH_ROS_API ChROSGPSHandler : public ChROSHandler {
  public:
    /// Tick at the sensor's own update rate.
    ChROSGPSHandler(std::shared_ptr<chrono::sensor::ChGPSSensor> gps, const std::string& topic_name);
    /// Tick at an explicit rate.
    ChROSGPSHandler(double update_rate,
                    std::shared_ptr<chrono::sensor::ChGPSSensor> gps,
                    const std::string& topic_name);

    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::array<double, 9> CalculateCovariance(const chrono::sensor::GPSData& gps_data);

    std::shared_ptr<chrono::sensor::ChGPSSensor> m_gps;
    const std::string m_topic_name;
    std::shared_ptr<ChROSPublisher> m_publisher;

    std::array<double, 3> m_running_average;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
