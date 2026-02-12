// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// ROS Handler for communicating accelerometer information
//
// =============================================================================

#ifndef CH_ROS_ACCELEROMETER_HANDLER
#define CH_ROS_ACCELEROMETER_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler_ipc.h"

#include <array>

namespace chrono {
namespace ros {

class ChROSIMUHandler;

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChAccelerometerSensor to ROS. Will publish sensor_msgs::msg::Imu.
class CH_ROS_API ChROSAccelerometerHandler : public ChROSHandler {
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

    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::ACCELEROMETER_DATA; }

    /// Get the serialized data for the handler
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    /// Helper function to calculate the covariance of the accelerometer
    /// ChAccelerometerSensor currently doesn't support covariance, so we'll use store
    /// the rolling averages and calculate the covariance here.
    std::array<double, 9> CalculateCovariance(const chrono::sensor::AccelData& imu_data);

  private:
    std::shared_ptr<chrono::sensor::ChAccelerometerSensor> m_imu;  ///< handle to the imu sensor

    const std::string m_topic_name;                                   ///< name of the topic to publish to

    std::array<double, 3> m_running_average;  ///< running average to calcualte covariance of the accelerometer

    // Cache for idempotency and friend access
    double m_last_time = -1.0;
    std::vector<uint8_t> m_last_serialized_data;
    ipc::AccelerometerData m_last_data_struct;

    friend class ChROSIMUHandler;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
