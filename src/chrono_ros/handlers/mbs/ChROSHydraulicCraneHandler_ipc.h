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
// Authors: Patrick Chen
// =============================================================================
//
// IPC data structures for the hydraulic crane co-simulation handlers.
// Shared between the main Chrono process and the ROS subprocess.
//
// This file must remain free of all ROS and Chrono headers -- only plain
// C++ types are allowed so that both sides can include it safely.
//
// =============================================================================

#ifndef CH_ROS_HYDRAULIC_CRANE_HANDLER_IPC_H
#define CH_ROS_HYDRAULIC_CRANE_HANDLER_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

/// Crane mechanism state transmitted from the main process to the ROS subprocess.
/// Published as std_msgs/Float64MultiArray with the layout [s, sd].
struct CraneStateData {
    char topic_name[128];  ///< ROS topic for publishing crane state
    double s;              ///< Actuator length [m]
    double sd;             ///< Actuator length rate [m/s]
};

/// Hydraulic actuator state transmitted from the main process to the ROS subprocess.
/// Published as std_msgs/Float64MultiArray with the layout
/// [force, valve_pos, pressure_0, pressure_1, Uref].
struct ActuatorStateData {
    char topic_name[128];  ///< ROS topic for publishing actuator state
    double force;          ///< Actuator force [N]
    double valve_position; ///< Directional valve spool position [-]
    double pressure_0;     ///< Cylinder chamber 0 pressure [Pa]
    double pressure_1;     ///< Cylinder chamber 1 pressure [Pa]
    double Uref;           ///< Actuation reference input [-]
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
