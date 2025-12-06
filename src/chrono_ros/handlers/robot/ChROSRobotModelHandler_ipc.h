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
// IPC data structure for RobotModelHandler
//
// =============================================================================

#ifndef CH_ROS_ROBOT_MODEL_HANDLER_IPC_H
#define CH_ROS_ROBOT_MODEL_HANDLER_IPC_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"

namespace chrono {
namespace ros {
namespace ipc {

struct RobotModelData {
    char topic_name[128];
    uint32_t model_length;
    // The robot model string follows immediately after this struct
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
