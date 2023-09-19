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
// Author: Aaron Young
// =============================================================================
//
// Handler for interfacing a ChDriver to ROS
//
// =============================================================================

#ifndef CH_ROS_DRIVER_INPUTS_HANDLER
#define CH_ROS_DRIVER_INPUTS_HANDLER

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros_interfaces/msg/driver_inputs.hpp"

#include "chrono_vehicle/ChDriver.h"

#include "rclcpp/subscription.hpp"

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_vehicle_handlers
/// @{

/// This handler is responsible for interfacing a ChDriver to ROS. Will instantiate a subscriber to chrono_ros_interfaces::msg::DriverInputs on "~/input/driver_inputs".
class ChROSDriverInputsHandler : public ChROSHandler {
  public:
    /// Constructor for the ChROSDriverInputsHandler class. Takes a ChDriver. A subscriber will listen for data, store received data, and update the driver only during the Ticks.
    ChROSDriverInputsHandler(uint64_t frequency, std::shared_ptr<chrono::vehicle::ChDriver> driver);

    /// Initializes the handler. Creates a subscriber of chrono_ros_interfaces::msg::DriverInputs on topic "~/input/driver_inputs".
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored inputs data from Callback
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const chrono_ros_interfaces::msg::DriverInputs& msg);

  private:
    chrono::vehicle::DriverInputs m_inputs;
    std::shared_ptr<chrono::vehicle::ChDriver> m_driver;

    rclcpp::Subscription<chrono_ros_interfaces::msg::DriverInputs>::SharedPtr m_subscription;

    std::mutex m_mutex; ///< used to control access to m_inputs
};

}  // namespace ros
}  // namespace chrono

#endif
