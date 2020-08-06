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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

namespace chrono {
namespace sensor {

/// @addtogroup sensor_cuda
/// @{

/// Function for reduction of data when multiple samples are used per beam. The performs a mean average of the data with
/// the sample radius.
/// @param buf Input/output device pointer to raw lidar data. Computation will be done in-place
/// @param width Width of the input data
/// @param height Height of the inpute data
/// @param threshold Intensity threshold for removing points
/// @param default_dist Default distance to use when removing points
void cuda_lidar_clip(float* buf, int width, int height, float threshold, float default_dist);

/// @}

}  // namespace sensor
}  // namespace chrono
