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
// IPC data structure for CameraHandler
// This file contains ONLY the IPC struct - no ROS or Chrono dependencies
// Safe to include in both main process and subprocess
//
// =============================================================================

#ifndef CH_ROS_CAMERA_HANDLER_IPC_H
#define CH_ROS_CAMERA_HANDLER_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

/// Camera image metadata for IPC communication
/// Image pixel data follows this header in the serialized message
struct CameraData {
    char topic_name[128];  ///< ROS topic name for publishing
    char frame_id[64];     ///< Frame ID for the image
    char encoding[64];     ///< Image encoding format, such as "rgba8" or "32FC1"
    uint32_t width;        ///< Image width in pixels
    uint32_t height;       ///< Image height in pixels
    uint32_t step;         ///< Row stride in bytes (width * bytes_per_pixel)
    // Pixel data follows immediately after this struct in serialized buffer
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
