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
/// @param max_v_angle The maximun vertical fov angle of the lidar
/// @param min_v_angle The minimum vertical fov angle for the lidar
/// @param stream The cuda stream for the kernel
void cuda_pointcloud_from_depth(void* bufDI,
                                void* bufOut,
                                int width,
                                int height,
                                float hfov,
                                float max_v_angle,
                                float min_v_angle,
                                CUstream& stream);

/// Converts depth data to point cloud data for a lidar when the mode is dual return.
/// @param bufDI A device pointer to depth/intensity data from a lidar.
/// @param bufOut A device pointer to when the point cloud data will be stored.
/// @param width The width of the lidar data.
/// @param height The height of the lidar data.
/// @param hfov The horizontal field of view of the lidar.
/// @param max_v_angle The maximun vertical fov angle of the lidar
/// @param min_v_angle The minimum vertical fov angle for the lidar
/// @param stream The cuda stream for the kernel
void cuda_pointcloud_from_depth_dual_return(void* bufDI,
                                            void* bufOut,
                                            int width,
                                            int height,
                                            float hfov,
                                            float max_v_angle,
                                            float min_v_angle,
                                            CUstream& stream);

/// @}
}  // namespace sensor
}  // namespace chrono
