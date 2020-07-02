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
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <chrono>
#include <memory>

namespace chrono {
namespace sensor {

// cuda_camera_noise_pixel_dependent
bool init_camera_noise_const_normal = true;
std::shared_ptr<curandState_t> camera_noise_const_normal_rng;

// cuda_camera_noise_pixel_dependent
bool init_camera_noise_pix_dep = true;
std::shared_ptr<curandState_t> camera_noise_pix_dep_rng;

// cuda_camera_noise_crf
bool init_camera_noise_crf = true;
std::shared_ptr<curandState_t> camera_noise_crf_rng;

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

        // convert back to char and save in image
        bufPtr[index * 4] = (unsigned char)(r * 255.999);
        bufPtr[index * 4 + 1] = (unsigned char)(g * 255.999);
        bufPtr[index * 4 + 2] = (unsigned char)(b * 255.999);
    }
}

__global__ void pix_dep_noise_kernel(unsigned char* bufPtr,
                                     int w,
                                     int h,
                                     float gain,
                                     float sigma_shot,
                                     float sigma_adc,
                                     curandState_t* rng_states) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;

    if (index < w * h) {
        // get pixel values
        unsigned char pix_r = bufPtr[index * 4];
        unsigned char pix_g = bufPtr[index * 4 + 1];
        unsigned char pix_b = bufPtr[index * 4 + 2];

        float r = ((float)(pix_r)) / 255.0;
        float g = ((float)(pix_g)) / 255.0;
        float b = ((float)(pix_b)) / 255.0;

        // curand_normal(&rng_states[index]);
        float stdev_r = sqrtf((r * sigma_shot * sigma_shot) + (sigma_adc * sigma_adc));
        float stdev_g = sqrtf((g * sigma_shot * sigma_shot) + (sigma_adc * sigma_adc));
        float stdev_b = sqrtf((b * sigma_shot * sigma_shot) + (sigma_adc * sigma_adc));
        float r_rand = curand_normal(&rng_states[index]) * stdev_r;
        float g_rand = curand_normal(&rng_states[index]) * stdev_g;
        float b_rand = curand_normal(&rng_states[index]) * stdev_b;

        // convert to float and add noise
        r = r + r_rand;
        g = g + g_rand;
        b = b + b_rand;

        // convert back to char and save in image
        bufPtr[index * 4] = (unsigned char)(r * 255.999);
        bufPtr[index * 4 + 1] = (unsigned char)(g * 255.999);
        bufPtr[index * 4 + 2] = (unsigned char)(b * 255.999);
    }
}

void cuda_camera_noise_const_normal(void* bufPtr, int width, int height, float mean, float stdev) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;

    if (init_camera_noise_const_normal) {
        camera_noise_const_normal_rng = std::shared_ptr<curandState_t>(
            cudaMallocHelper<curandState_t>(nBlocks * nThreads), cudaFreeHelper<curandState_t>);
        init_camera_noise_const_normal = false;
        init_random_states<<<nBlocks, nThreads>>>(std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                                                  camera_noise_const_normal_rng.get(), nBlocks * nThreads);
    }

    const_normal_noise_kernel<<<nBlocks, nThreads>>>((unsigned char*)bufPtr, width, height, mean, stdev,
                                                     camera_noise_const_normal_rng.get());
}

void cuda_camera_noise_pixel_dependent(void* bufPtr,
                                       int width,
                                       int height,
                                       float gain,
                                       float sigma_read,
                                       float sigma_adc) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;

    if (init_camera_noise_pix_dep) {
        camera_noise_pix_dep_rng = std::shared_ptr<curandState_t>(cudaMallocHelper<curandState_t>(nBlocks * nThreads),
                                                                  cudaFreeHelper<curandState_t>);
        init_camera_noise_pix_dep = false;
        init_random_states<<<nBlocks, nThreads>>>(std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                                                  camera_noise_pix_dep_rng.get(), nBlocks * nThreads);
    }

    pix_dep_noise_kernel<<<nBlocks, nThreads>>>((unsigned char*)bufPtr, width, height, gain, sigma_read, sigma_adc,
                                                camera_noise_pix_dep_rng.get());
}

}  // namespace sensor
}  // namespace chrono
