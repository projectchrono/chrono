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

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

#include "curand_utils.cuh"
#include "camera_noise.cuh"
// #include "chrono_sensor/utils/CudaMallocHelper.h"

#include "chrono_sensor/optix/shaders/device_utils.h"

// #include <chrono>
// #include <memory>

namespace chrono {
namespace sensor {

// Add random normal noise to the image with constant mean and stdev
__global__ void const_normal_noise_kernel(unsigned char* bufPtr,
                                          int w,
                                          int h,
                                          float mean,
                                          float stdev,
                                          curandState_t* rng_states) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;

    if (index < w * h) {
        // curand_normal(&rng_states[index]);
        float r_rand = curand_normal(&rng_states[index]) * stdev + mean;
        float g_rand = curand_normal(&rng_states[index]) * stdev + mean;
        float b_rand = curand_normal(&rng_states[index]) * stdev + mean;

        // get pixel values
        unsigned char pix_r = bufPtr[index * 4];
        unsigned char pix_g = bufPtr[index * 4 + 1];
        unsigned char pix_b = bufPtr[index * 4 + 2];

        // convert to float and add noise
        float r = ((float)(pix_r)) / 255.0 + r_rand;
        float g = ((float)(pix_g)) / 255.0 + g_rand;
        float b = ((float)(pix_b)) / 255.0 + b_rand;

        // prevent overflow
        r = clamp(r, 0.f, 1.f);
        g = clamp(g, 0.f, 1.f);
        b = clamp(b, 0.f, 1.f);

        // convert back to char and save in image
        bufPtr[index * 4] = (unsigned char)(r * 255.999);
        bufPtr[index * 4 + 1] = (unsigned char)(g * 255.999);
        bufPtr[index * 4 + 2] = (unsigned char)(b * 255.999);
    }
}

__global__ void pix_dep_noise_kernel(unsigned char* bufPtr,
                                     int w,
                                     int h,
                                     float variance_slope,
                                     float variance_intercept,
                                     curandState_t* rng_states) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;

    if (index < w * h) {
        // get pixel values
        unsigned char pix_r = bufPtr[index * 4];
        unsigned char pix_g = bufPtr[index * 4 + 1];
        unsigned char pix_b = bufPtr[index * 4 + 2];

        float r = ((float)(pix_r)) / 255.f;
        float g = ((float)(pix_g)) / 255.f;
        float b = ((float)(pix_b)) / 255.f;

        // curand_normal(&rng_states[index]);
        float stdev_r = sqrtf(r * variance_slope + variance_intercept);
        float stdev_g = sqrtf(g * variance_slope + variance_intercept);
        float stdev_b = sqrtf(b * variance_slope + variance_intercept);
        float r_rand = curand_normal(&rng_states[index]) * stdev_r;
        float g_rand = curand_normal(&rng_states[index]) * stdev_g;
        float b_rand = curand_normal(&rng_states[index]) * stdev_b;

        // convert to float and add noise (prevent overflow)
        r = clamp(r + r_rand, 0.f, 1.f);
        g = clamp(g + g_rand, 0.f, 1.f);
        b = clamp(b + b_rand, 0.f, 1.f);

        // convert back to char and save in image
        bufPtr[index * 4] = (unsigned char)(r * 255.999);
        bufPtr[index * 4 + 1] = (unsigned char)(g * 255.999);
        bufPtr[index * 4 + 2] = (unsigned char)(b * 255.999);
    }
}

void cuda_camera_noise_const_normal(unsigned char* bufPtr,
                                    int width,
                                    int height,
                                    float mean,
                                    float stdev,
                                    curandState_t* rng,
                                    CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;

    const_normal_noise_kernel<<<nBlocks, nThreads, 0, stream>>>(bufPtr, width, height, mean, stdev, rng);
}

void cuda_camera_noise_pixel_dependent(unsigned char* bufPtr,
                                       int width,
                                       int height,
                                       float variance_slope,
                                       float variance_intercept,
                                       curandState_t* rng,
                                       CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;

    pix_dep_noise_kernel<<<nBlocks, nThreads, 0, stream>>>(bufPtr, width, height, variance_slope, variance_intercept, rng);
}

}  // namespace sensor
}  // namespace chrono
