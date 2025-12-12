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
// ROS Handler for communicating gyroscope information
//
// =============================================================================

#ifndef CH_ROS_GYROSCOPE_HANDLER
#define CH_ROS_GYROSCOPE_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler_ipc.h"

#include <array>

namespace chrono {
namespace ros {

class ChROSIMUHandler;

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChGyroscopeSensor to ROS. Will publish sensor_msgs::msg::Imu.
class CH_ROS_API ChROSGyroscopeHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to imu->GetUpdateRate().
    /// The update rate corresponds to the sensor's update rate.
    ChROSGyroscopeHandler(std::shared_ptr<chrono::sensor::ChGyroscopeSensor> imu, const std::string& topic_name);

    /// Full constructor. Takes a ChGyroscopeSensor, update rate, and topic name.
    ChROSGyroscopeHandler(double update_rate,
                          std::shared_ptr<chrono::sensor::ChGyroscopeSensor> imu,
                          const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::GYROSCOPE_DATA; }

    /// Get the serialized data for the handler
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    /// Helper function to calculate the covariance of the gyroscope
    /// ChGyroscopeSensor currently doesn't support covariance, so we'll use store
    /// the running average of the gyroscope data to calculate the covariance
    std::array<double, 9> CalculateCovariance(const chrono::sensor::GyroData& imu_data);

    std::shared_ptr<chrono::sensor::ChGyroscopeSensor> m_imu;
    const std::string m_topic_name;
    std::array<double, 3> m_running_average;

    // Cache for idempotency and friend access
    double m_last_time = -1.0;
    std::vector<uint8_t> m_last_serialized_data;
    ipc::GyroscopeData m_last_data_struct;

    friend class ChROSIMUHandler;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif


#ifndef CH_ROS_GYROSCOPE_HANDLER
#define CH_ROS_GYROSCOPE_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler_ipc.h"

#include <array>

namespace chrono {
namespace ros {

class ChROSIMUHandler;

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChGyroscopeSensor to ROS. Will publish sensor_msgs::msg::Imu.
class CH_ROS_API ChROSGyroscopeHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to imu->GetUpdateRate().
    /// The update rate corresponds to the sensor's update rate.
    ChROSGyroscopeHandler(std::shared_ptr<chrono::sensor::ChGyroscopeSensor> imu, const std::string& topic_name);

    /// Full constructor. Takes a ChGyroscopeSensor, update rate, and topic name.
    ChROSGyroscopeHandler(double update_rate,
                          std::shared_ptr<chrono::sensor::ChGyroscopeSensor> imu,
                          const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::GYROSCOPE_DATA; }

    /// Get the serialized data for the handler
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    /// Helper function to calculate the covariance of the gyroscope
    /// ChGyroscopeSensor currently doesn't support covariance, so we'll use store
    /// the running average of the gyroscope data to calculate the covariance
    std::array<double, 9> CalculateCovariance(const chrono::sensor::GyroData& imu_data);

    std::shared_ptr<chrono::sensor::ChGyroscopeSensor> m_imu;
    const std::string m_topic_name;
    std::array<double, 3> m_running_average;

    // Cache for idempotency and friend access
    double m_last_time = -1.0;
    std::vector<uint8_t> m_last_serialized_data;
    ipc::GyroscopeData m_last_data_struct;

    friend class ChROSIMUHandler;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
