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

/// Kernel for applying uniform Gaussian noise to an image.
/// @param bufPtr A uchar (values 0-255) pointer to device memory where the image is stored. Memory assumed to be row
/// major and contiguous. (Flat array representing image).
/// @param width The width of the image in pixels.
/// @param height The height of the image in pixels.
/// @param mean The mean of the Gaussian distribution to be sampled.
/// @param stdev  The standard deviation of the distribution to be sampled.
void cuda_camera_noise_const_normal(unsigned char* bufPtr,
                                    int width,
                                    int height,
                                    float mean,
                                    float stdev,
                                    curandState_t* rng);

/// Kernel for applying pixel dependent Gaussian noise to an image.
/// @param bufPtr A uchar (values 0-255) pointer to device memory where the image is stored. Memory assumed to be row
/// major and contiguous. (Flat array representing image).
/// @param width The width of the image in pixels.
/// @param height The height of the image in pixels.
/// @param gain The linear coefficient of correlation between pixel intensity and noise variance
/// @param sigma_read The standard deviation of multiplicative noise to be sampled.
/// @param sigma_adc The standard deviation of additive noise to be sampled.
void cuda_camera_noise_pixel_dependent(unsigned char* bufPtr,
                                       int width,
                                       int height,
                                       float gain,
                                       float sigma_read,
                                       float sigma_adc,
                                       curandState_t* rng);

/// @}

}  // namespace sensor
}  // namespace chrono
