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

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

#include "curand_utils.cuh"
#include "lidar_noise.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <chrono>
#include <memory>

namespace chrono {
namespace sensor {

// Add random normal noise to the image with constant mean and stdev
__global__ void lidar_normal_noise_kernel(float* bufPtr,
                                          int w,
                                          int h,
                                          float stdev_range,
                                          float stdev_v_angle,
                                          float stdev_h_angle,
                                          float stdev_intensity,
                                          curandState_t* rng_states) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;

    if (index < w * h) {
        // get the intensity from the buffer
        float i = bufPtr[index * 4 + 3];

        if (i > 1e-6) {
            // get values from the buffer
            float x = bufPtr[index * 4];
            float y = bufPtr[index * 4 + 1];
            float z = bufPtr[index * 4 + 2];

            // convert to spherical coordinates
            float range = sqrt(x * x + y * y + z * z);
            // small values here to prevent div by 0 and to prevent acos and asin outside valid ranges
            if (range > 1e-6) {
                float phi = asin(z / (range + 1e-6));
                float theta = acos(x / ((range + 1e-6) * cos(phi)));

                // get correct sign on theta
                if (y < 0)
                    theta = -theta;

                // apply noise
                range += curand_normal(&rng_states[index]) * stdev_range;
                theta += curand_normal(&rng_states[index]) * stdev_h_angle;
                phi += curand_normal(&rng_states[index]) * stdev_v_angle;
                i += curand_normal(&rng_states[index]) * stdev_intensity;
                i = i > 0 ? i : 0;

                // convert back to XZY
                z = sin(phi) * range;
                y = sin(theta) * cos(phi) * range;
                x = cos(theta) * cos(phi) * range;

                // save into buffer
                bufPtr[index * 4] = x;
                bufPtr[index * 4 + 1] = y;
                bufPtr[index * 4 + 2] = z;
                bufPtr[index * 4 + 3] = i > 0 ? i : 0;
            }
        }
    }
}

void cuda_lidar_noise_normal(float* bufPtr,
                             int width,
                             int height,
                             float stdev_range,
                             float stdev_v_angle,
                             float stdev_h_angle,
                             float stdev_intensity,
                             curandState_t* rng) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;

    lidar_normal_noise_kernel<<<nBlocks, nThreads>>>(bufPtr, width, height, stdev_range, stdev_v_angle, stdev_h_angle,
                                                     stdev_intensity, rng);
}

}  // namespace sensor
}  // namespace chrono
