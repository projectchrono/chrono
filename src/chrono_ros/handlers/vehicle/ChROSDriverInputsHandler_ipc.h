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
// IPC data structure for DriverInputsHandler
// This file contains ONLY the IPC struct - no ROS or Chrono dependencies
// Safe to include in both main process and subprocess
//
// =============================================================================

#ifndef CH_ROS_DRIVER_INPUTS_HANDLER_IPC_H
#define CH_ROS_DRIVER_INPUTS_HANDLER_IPC_H

namespace chrono {
namespace ros {
namespace ipc {

/// Driver inputs IPC data structure for bidirectional communication
/// This struct is sent FROM subprocess TO main process when ROS messages arrive
/// Requirements:
/// - Plain C++ types only (no STL, no pointers, no ROS types, no Chrono types)
/// - Fixed size for reliable serialization
/// - Trivially copyable (use memcpy)
struct DriverInputsData {
    double steering;   ///< Steering angle [-1, 1]
    double throttle;   ///< Throttle position [0, 1]
    double braking;    ///< Braking intensity [0, 1]
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
