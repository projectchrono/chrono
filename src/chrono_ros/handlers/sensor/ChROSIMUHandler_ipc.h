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
// IPC data structure for IMUHandler
//
// =============================================================================

#ifndef CH_ROS_IMU_HANDLER_IPC_H
#define CH_ROS_IMU_HANDLER_IPC_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"

namespace chrono {
namespace ros {
namespace ipc {

struct IMUData {
    char topic_name[128];
    char frame_id[128];
    
    bool has_accel;
    double linear_acceleration[3];
    double linear_acceleration_covariance[9];
    
    bool has_gyro;
    double angular_velocity[3];
    double angular_velocity_covariance[9];
    
    bool has_mag;
    double orientation[4]; // xyzw
    double orientation_covariance[9];
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
