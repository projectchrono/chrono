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

#include "sensor_msgs/msg/imu.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChAccelerometerSensor to ROS. Will publish sensor_msgs::msg::Imu.
class ChROSAccelerometerHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to imu->GetUpdateRate().
    /// The update rate corresponds to the sensor's update rate.
    ChROSAccelerometerHandler(std::shared_ptr<chrono::sensor::ChAccelerometerSensor> imu,
                              const std::string& topic_name);

    /// Full constructor. Takes a ChAccelerometerSensor, update rate, and topic name.
    ChROSAccelerometerHandler(double update_rate,
                              std::shared_ptr<chrono::sensor::ChAccelerometerSensor> imu,
                              const std::string& topic_name);

    /// Initializes the handler. 
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChAccelerometerSensor> m_imu;  ///< handle to the imu sensor

    const std::string m_topic_name;                                   ///< name of the topic to publish to
    sensor_msgs::msg::Imu m_imu_msg;                                  ///< message to publish
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;  ///< the publisher for the imu message
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
