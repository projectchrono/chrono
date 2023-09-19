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
// ROS Handler for communicating magnetometer information
//
// =============================================================================

#ifndef CH_ROS_MAGNETOMETER_HANDLER
#define CH_ROS_MAGNETOMETER_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChMagnetometerSensor to ROS. Will publish sensor_msgs::msg::MagneticField.
class ChROSMagnetometerHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ChMagnetometerSensor.
    ChROSMagnetometerHandler(std::shared_ptr<chrono::sensor::ChMagnetometerSensor> imu);

    /// Initializes the handler. Creates a publisher for the magnetic field data on the topic "~/output/magnetometer/imu.GetName()/data"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChMagnetometerSensor> m_imu;

    sensor_msgs::msg::MagneticField m_mag_msg;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_publisher;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
