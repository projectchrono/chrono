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
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#ifndef CH_VIPER_DC_MOTOR_CONTROL_HANDLER_H
#define CH_VIPER_DC_MOTOR_CONTROL_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono_models/robot/viper/Viper.h"

#include "rclcpp/subscription.hpp"
#include "chrono_ros_interfaces/msg/viper_dc_motor_control.hpp"

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// This handler is responsible for interfacing a ViperDCMotorControl driver to ROS. Will instantiate a subscriber to chrono_ros_interfaces::msg::ViperDCMotorControl on "~/input/driver_inputs".
class ChROSViperDCMotorControlHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ViperDCMotorControl driver
    ChROSViperDCMotorControlHandler(uint64_t frequency, std::shared_ptr<chrono::viper::ViperDCMotorControl> driver);

    /// Initializes the handler. Creates a subscriber of chrono_ros_interfaces::msg::ViperDCMotorControl on topic "~/input/driver_inputs".
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored inputs data from Callback
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const chrono_ros_interfaces::msg::ViperDCMotorControl& msg);

  private:
    chrono_ros_interfaces::msg::ViperDCMotorControl m_msg;
    std::shared_ptr<chrono::viper::ViperDCMotorControl> m_driver;

    rclcpp::Subscription<chrono_ros_interfaces::msg::ViperDCMotorControl>::SharedPtr m_subscription;

    std::mutex m_mutex;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
