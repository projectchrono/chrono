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
// ROS Handler for communicating magnetometer information
//
// =============================================================================

#ifndef CH_ROS_MAGNETOMETER_HANDLER
#define CH_ROS_MAGNETOMETER_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler_ipc.h"

#include <array>

namespace chrono {
namespace ros {

class ChROSIMUHandler;

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChMagnetometerSensor to ROS. Will publish
/// sensor_msgs::msg::MagneticField.
class CH_ROS_API ChROSMagnetometerHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to imu->GetUpdateRate().
    ChROSMagnetometerHandler(std::shared_ptr<chrono::sensor::ChMagnetometerSensor> imu, const std::string& topic_name);

    /// Full constructor. Takes a ChMagnetometerSensor, update rate, and topic name.
    ChROSMagnetometerHandler(double update_rate,
                             std::shared_ptr<chrono::sensor::ChMagnetometerSensor> imu,
                             const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::MAGNETOMETER_DATA; }

    /// Get the serialized data for the handler
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    /// Helper function to calculate the covariance of the magnetometer
    /// ChMagnetometerSensor currently doesn't support covariance, so we'll use store
    /// the rolling averages and calculate the covariance here.
    std::array<double, 9> CalculateCovariance(const chrono::sensor::MagnetData& imu_data);

  private:
    std::shared_ptr<chrono::sensor::ChMagnetometerSensor> m_imu;  ///< handle to the imu sensor

    const std::string m_topic_name;                                             ///< name of the topic to publish to
    
    std::array<double, 3> m_running_average;  ///< rolling average of the magnetometer data for covariance calculation

    // Cache for idempotency and friend access
    double m_last_time = -1.0;
    std::vector<uint8_t> m_last_serialized_data;
    ipc::MagnetometerData m_last_data_struct;

    friend class ChROSIMUHandler;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
