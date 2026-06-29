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
// ROS handler that aggregates accelerometer + gyroscope (+ magnetometer) data
// into a single sensor_msgs/msg/Imu message.
//
// =============================================================================

#ifndef CH_ROS_IMU_HANDLER_H
#define CH_ROS_IMU_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;
class ChROSAccelerometerHandler;
class ChROSGyroscopeHandler;
class ChROSMagnetometerHandler;

/// @addtogroup ros_sensor_handlers
/// @{

/// Combines the latest data of an accelerometer + gyroscope (+ magnetometer)
/// handler into one sensor_msgs/msg/Imu. The sub-handlers are registered with
/// the manager separately (they tick and publish independently); this handler
/// just reads their most recent extracted values. Set the sub-handlers' rate
/// >= this handler's rate so their data is fresh.
class CH_ROS_API ChROSIMUHandler : public ChROSHandler {
  public:
    /// @param update_rate publish rate (Hz, sim time); 0 = every step.
    /// @param topic_name Imu topic to publish on.
    /// @param frame_id header frame_id stamped into the message.
    ChROSIMUHandler(double update_rate, const std::string& topic_name, const std::string& frame_id = "imu");

    /// Creates the Imu publisher.
    virtual bool Initialize(ChROSBridge& bridge) override;

    /// Set the accelerometer sub-handler (required). Its rate should be >= this handler's.
    void SetAccelerometerHandler(std::shared_ptr<ChROSAccelerometerHandler> accel_handler);
    /// Set the gyroscope sub-handler (required). Its rate should be >= this handler's.
    void SetGyroscopeHandler(std::shared_ptr<ChROSGyroscopeHandler> gyro_handler);
    /// Set the magnetometer sub-handler (required for presence, though its data is not packed).
    void SetMagnetometerHandler(std::shared_ptr<ChROSMagnetometerHandler> mag_handler);

  protected:
    /// Assembles one Imu message from the sub-handlers' latest values and publishes it.
    virtual void Tick(double time) override;

  private:
    const std::string m_topic_name;
    const std::string m_frame_id;
    std::shared_ptr<ChROSPublisher> m_publisher;

    std::shared_ptr<ChROSAccelerometerHandler> m_accel_handler;
    std::shared_ptr<ChROSGyroscopeHandler> m_gyro_handler;
    std::shared_ptr<ChROSMagnetometerHandler> m_mag_handler;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
