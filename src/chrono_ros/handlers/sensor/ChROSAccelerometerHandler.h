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
// ROS Handler for communicating accelerometer information
//
// =============================================================================

#ifndef CH_ROS_ACCELEROMETER_HANDLER
#define CH_ROS_ACCELEROMETER_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChAccelerometerSensor to ROS. Will publish sensor_msgs::msg::Imu.
class ChROSAccelerometerHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ChAccelerometerSensor.
    ChROSAccelerometerHandler(std::shared_ptr<chrono::sensor::ChAccelerometerSensor> imu);

    /// Initializes the handler. Creates a publisher for the imu data on the topic "~/output/accelerometer/imu.GetName()/data"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChAccelerometerSensor> m_imu;

    sensor_msgs::msg::Imu m_imu_msg;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
