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

/// RGB 8 bpp image of uchar to greyscale image conversion
/// @param bufRGBA  The input buffer of RGBA data in uchar format (8 bpp)
/// @param bufOut A pointer to the output buffer where greyscale data will be held (8 bpp)
/// @param width The width of the image
/// @param height The height of the image
void cuda_grayscale(void* bufRGBA, void* bufOut, int width, int height);

/// @}

}  // namespace sensor
}  // namespace chrono
