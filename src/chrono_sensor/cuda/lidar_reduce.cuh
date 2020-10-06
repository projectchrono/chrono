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
/// @param bufIn Input device pointed to raw lidar data.
/// @param bufOut Outout device pointer for processed lidar data.
/// @param width Width of the input data.
/// @param height Height of the inpute data.
/// @param radius Radius in samples of the beam to be reduced.
void cuda_lidar_mean_reduce(void* bufIn, void* bufOut, int width, int height, int radius);

/// Function for reduction of data when multiple samples are used per beam. The calculates the strongest return within
/// the sample radius of the beam.
/// @param bufIn Input device pointed to raw lidar data.
/// @param bufOut Outout device pointer for processed lidar data.
/// @param width Width of the input data.
/// @param height Height of the inpute data.
/// @param radius Radius in samples of the beam to be reduced.
void cuda_lidar_strong_reduce(void* bufIn, void* bufOut, int width, int height, int radius);

/// @}

}  // namespace sensor
}  // namespace chrono
