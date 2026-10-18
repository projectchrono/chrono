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
// ROS handler for a ChAccelerometerSensor (publishes sensor_msgs/msg/Imu).
//
// =============================================================================

#ifndef CH_ROS_ACCELEROMETER_HANDLER_H
#define CH_ROS_ACCELEROMETER_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono/core/ChVector3.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"

#include <array>
#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;
class ChROSIMUHandler;

/// @addtogroup ros_sensor_handlers
/// @{

/// Publishes a ChAccelerometerSensor as sensor_msgs/msg/Imu (only the linear
/// acceleration fields are populated).
class CH_ROS_API ChROSAccelerometerHandler : public ChROSHandler {
  public:
    /// Tick at the sensor's own update rate.
    ChROSAccelerometerHandler(std::shared_ptr<chrono::sensor::ChAccelerometerSensor> imu,
                              const std::string& topic_name);
    /// Tick at an explicit rate.
    ChROSAccelerometerHandler(double update_rate,
                              std::shared_ptr<chrono::sensor::ChAccelerometerSensor> imu,
                              const std::string& topic_name);

    /// Creates the Imu publisher.
    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    /// Reads the accelerometer buffer and publishes the linear acceleration.
    virtual void Tick(double time) override;

  private:
    std::array<double, 9> CalculateCovariance(const chrono::sensor::AccelData& data);

    std::shared_ptr<chrono::sensor::ChAccelerometerSensor> m_imu;
    const std::string m_topic_name;
    std::shared_ptr<ChROSPublisher> m_publisher;

    std::array<double, 3> m_running_average;

    // Last-extracted data, retained for the composite ChROSIMUHandler to read.
    chrono::ChVector3d m_linear_acceleration;
    std::array<double, 9> m_linear_acceleration_covariance;

    friend class ChROSIMUHandler;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
