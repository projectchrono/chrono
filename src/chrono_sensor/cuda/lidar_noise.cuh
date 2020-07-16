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

/// Function for applying Gaussian noise to raw ground truth lidar data. Independent for range, intensity, and angles
/// @param bufPtr Device pointer to depth/intensity lidar data.
/// @param width Width of lidar data buffer.
/// @param height Height of lidar data buffer.
/// @param stdev_range Standard deviation for lidar range.
/// @param stdev_h_angle Standard deviation of noise for horizontal angle measurement.
/// @param stdev_intensity Standard deviation of noise for vertical angle measurement.
void cuda_lidar_noise_normal(float* bufPtr,
                             int width,
                             int height,
                             float stdev_range,
                             float stdev_v_angle,
                             float stdev_h_angle,
                             float stdev_intensity,
                             curandState_t* rng);

/// @}

}  // namespace sensor
}  // namespace chrono
