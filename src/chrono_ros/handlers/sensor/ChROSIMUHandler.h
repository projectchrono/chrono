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
// ROS Handler for communicating imu information. Packages the data from other
// imu handlers into a single message and publishes it.
//
// =============================================================================

#ifndef CHRONO_ROS_SENSOR_IMU_HANDLER_H
#define CHRONO_ROS_SENSOR_IMU_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace chrono {
namespace ros {

    class ChROSAccelerometerHandler;
    class ChROSGyroscopeHandler;
    class ChROSMagnetometerHandler;

class ChROSIMUHandler : public ChROSHandler {
  public:
    ChROSIMUHandler(double update_rate, const std::string& topic_name, const std::string& frame_id = "imu");

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Set the imu sensor handlers
    /// NOTE: The handlers will tick at the same rate as the imu handler
    void SetAccelerometerHandler(std::shared_ptr<ChROSAccelerometerHandler> accel_handler);
    void SetGyroscopeHandler(std::shared_ptr<ChROSGyroscopeHandler> gyro_handler);
    void SetMagnetometerHandler(std::shared_ptr<ChROSMagnetometerHandler> mag_handler);

  protected:
    virtual void Tick(double time) override;

  private:
    const std::string m_topic_name;                                   ///< name of the topic to publish to
    const std::string m_frame_id;                                     ///< frame id of the imu
    sensor_msgs::msg::Imu m_imu_msg;                                  ///< message to publish
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;  ///< the publisher for the imu message

    std::shared_ptr<ChROSAccelerometerHandler> m_accel_handler;
    std::shared_ptr<ChROSGyroscopeHandler> m_gyro_handler;
    std::shared_ptr<ChROSMagnetometerHandler> m_mag_handler;
};

}  // namespace ros
}  // namespace chrono

#endif