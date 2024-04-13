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

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_vehicle_handlers
/// @{

/// This handler is responsible for interfacing a ChDriver to ROS. Will instantiate a subscriber to
/// chrono_ros_interfaces::msg::DriverInputs 
class ChROSDriverInputsHandler : public ChROSHandler {
  public:
    /// Convenience constructor. Will set the update rate to 0, which means the Tick()
    /// function will update on each update call.
    ChROSDriverInputsHandler(std::shared_ptr<chrono::vehicle::ChDriver> driver, const std::string& topic_name);

    /// Constructor for the ChROSDriverInputsHandler class. Takes a ChDriver. A subscriber will listen for data, store
    /// received data, and update the driver only during the Ticks.
    ChROSDriverInputsHandler(double update_rate,
                             std::shared_ptr<chrono::vehicle::ChDriver> driver,
                             const std::string& topic_name);

    /// Initializes the handler. Creates a subscriber of chrono_ros_interfaces::msg::DriverInputs on topic
    /// "~/input/driver_inputs".
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored inputs data from Callback
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const chrono_ros_interfaces::msg::DriverInputs& msg);

  private:
    std::shared_ptr<chrono::vehicle::ChDriver> m_driver;  ///< the driver to update

    const std::string m_topic_name;          ///< name of the topic to publish to
    chrono::vehicle::DriverInputs m_inputs;  ///< stores the most recent inputs
    rclcpp::Subscription<chrono_ros_interfaces::msg::DriverInputs>::SharedPtr
        m_subscription;  ///< subscriber to the chrono_ros_interfaces::msg::DriverInputs

    std::mutex m_mutex;  ///< used to control access to m_inputs
};

}  // namespace ros
}  // namespace chrono

#endif
