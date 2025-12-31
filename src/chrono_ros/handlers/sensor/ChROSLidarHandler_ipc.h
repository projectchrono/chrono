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
// IPC data structure for LidarHandler
//
// =============================================================================

#ifndef CH_ROS_LIDAR_HANDLER_IPC_H
#define CH_ROS_LIDAR_HANDLER_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

struct LidarPointCloudData {
    char topic_name[128];
    char frame_id[64];
    uint32_t width;
    uint32_t height;
    // Data follows:
    // - Point data (float x, y, z, intensity) * width * height
};

struct LidarLaserScanData {
    char topic_name[128];
    char frame_id[64];
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    uint32_t count;
    // Data follows:
    // - Ranges (float) * count
    // - Intensities (float) * count
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
