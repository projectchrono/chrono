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
// Handler that drives a Viper ViperDCMotorControl from ROS
// (chrono_ros_interfaces/msg/ViperDCMotorControl).
//
// =============================================================================

#ifndef CH_ROS_VIPER_DC_MOTOR_CONTROL_HANDLER_H
#define CH_ROS_VIPER_DC_MOTOR_CONTROL_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono_models/robot/viper/Viper.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

class ChROSSubscription;

/// @addtogroup ros_robot_handlers
/// @{

/// Subscribes to chrono_ros_interfaces/msg/ViperDCMotorControl and applies the
/// received steering / lifting / motor-parameter commands to a Viper
/// ViperDCMotorControl driver. Another example of custom message-package support
/// (addressed by type-name string; no compiled message types). Call interface
/// matches 9.0. Wrapped for Python (pychrono.ros): the ViperDCMotorControl
/// argument crosses from pychrono.robot via SWIG cross-module type sharing.
class CH_ROS_API ChROSViperDCMotorControlHandler : public ChROSHandler {
  public:
    ChROSViperDCMotorControlHandler(double update_rate,
                                    std::shared_ptr<chrono::viper::ViperDCMotorControl> driver,
                                    const std::string& topic_name);

    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::viper::ViperDCMotorControl> m_driver;
    const std::string m_topic_name;
    std::shared_ptr<ChROSSubscription> m_subscription;

    // Most recent commands (written in the callback, applied in Tick; both run on
    // the simulation thread inside ChROSManager::Update(), so no lock - cf. I5).
    struct SteeringCommand {
        double angle;
        uint8_t wheel_id;  // ViperWheelID value; V_UNDEFINED (=4) means "all wheels"
    };
    std::vector<SteeringCommand> m_steering;
    double m_lifting = 0;
    double m_stall_torque = 0;
    uint8_t m_stall_wheel = 0;
    double m_no_load_speed = 0;
    uint8_t m_no_load_wheel = 0;
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif
