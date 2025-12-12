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
// IPC data structure for ViperDCMotorControlHandler
// This file contains ONLY the IPC struct - no ROS or Chrono dependencies
// Safe to include in both main process and subprocess
//
// =============================================================================

#ifndef CH_ROS_VIPER_DC_MOTOR_CONTROL_HANDLER_IPC_H
#define CH_ROS_VIPER_DC_MOTOR_CONTROL_HANDLER_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

/// Viper DC motor control IPC data structure for bidirectional communication
/// This struct is sent FROM subprocess TO main process when ROS messages arrive
/// Requirements:
/// - Plain C++ types only (no STL, no pointers, no ROS types, no Chrono types)
/// - Fixed size for reliable serialization
/// - Trivially copyable (use memcpy)
struct ViperDCMotorControlData {
    double steering_angle;           ///< Steering angle
    int32_t steering_wheel_id;       ///< Which wheel to apply steering to
    double lifting;                  ///< Lifting amount
    double stall_torque;             ///< Stall torque value
    int32_t stall_torque_wheel_id;   ///< Which wheel to apply stall torque to
    double no_load_speed;            ///< No-load speed value
    int32_t no_load_speed_wheel_id;  ///< Which wheel to apply no-load speed to
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
