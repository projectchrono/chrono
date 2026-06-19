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
// Handler that drives a ChDriver from ROS (chrono_ros_interfaces/msg/DriverInputs).
//
// =============================================================================

#ifndef CH_ROS_DRIVER_INPUTS_HANDLER_H
#define CH_ROS_DRIVER_INPUTS_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono_vehicle/ChDriver.h"

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSSubscription;

/// @addtogroup ros_vehicle_handlers
/// @{

/// Subscribes to chrono_ros_interfaces/msg/DriverInputs and applies the received
/// steering/throttle/braking to a ChVehicle ChDriver. This exercises a CUSTOM
/// ROS message package (chrono_ros_interfaces) end-to-end through the schema
/// bridge - addressed purely by type-name string, no compiled message types in
/// Chrono. Call interface matches 9.0.
///
/// Wrapped for Python (pychrono.ros): the ChDriver argument crosses from
/// pychrono.vehicle via SWIG's cross-module type sharing - see demo_ROS_vehicle.py.
class CH_ROS_API ChROSDriverInputsHandler : public ChROSHandler {
  public:
    /// Tick every simulation step (update_rate 0).
    ChROSDriverInputsHandler(std::shared_ptr<chrono::vehicle::ChDriver> driver, const std::string& topic_name);
    /// Tick at an explicit rate.
    ChROSDriverInputsHandler(double update_rate,
                             std::shared_ptr<chrono::vehicle::ChDriver> driver,
                             const std::string& topic_name);

    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::vehicle::ChDriver> m_driver;
    const std::string m_topic_name;
    std::shared_ptr<ChROSSubscription> m_subscription;

    // Most recent inputs (written in the subscription callback, applied in Tick;
    // both run on the simulation thread inside ChROSManager::Update(), so no lock).
    double m_steering = 0;
    double m_throttle = 0;
    double m_braking = 0;
};

/// @} ros_vehicle_handlers

}  // namespace ros
}  // namespace chrono

#endif
