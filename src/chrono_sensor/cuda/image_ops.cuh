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

/// An antialiasing helper function that reduces image quality by performing mean reduction of image
/// @param bufIn  A device pointer to the image image.
/// @param bufOut A device pointer to the ouput image.
/// @param w_out The output image width.
/// @param h_out The output image height.
/// @param factor The reduction factor per dimension.
/// @param pix_size Size of a pixel (number of channels).
void cuda_image_alias(void* bufIn, void* bufOut, int w_out, int h_out, int factor, int pix_size);

/// An image blurring function that reduces performs Gaussian blur.
/// @param buf
/// @param w
/// @param h
/// @param c
/// @param factor
void cuda_image_gauss_blur_char(void* buf, int w, int h, int c, int factor);

/// @}

}  // namespace sensor
}  // namespace chrono
