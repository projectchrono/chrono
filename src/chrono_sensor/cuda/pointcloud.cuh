// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================

namespace chrono {
namespace sensor {

/// @addtogroup sensor_cuda
/// @{

/// Converts depth data to point cloud data for a lidar.
/// @param bufDI A device pointer to depth/intensity data from a lidar.
/// @param bufOut A device pointer to when the point cloud data will be stored.
/// @param width The width of the lidar data.
/// @param height The height of the lidar data.
/// @param hfov The horizontal field of view of the lidar.
/// @param vfov The vertical field of view of the lidar.
void cuda_pointcloud_from_depth(void* bufDI,
                                void* bufOut,
                                int width,
                                int height,
                                float hfov,
                                float max_v_angle,
                                float min_v_angle);

/// @}
}  // namespace sensor
}  // namespace chrono
