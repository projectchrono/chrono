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
#include "grayscale.cuh"
#include <iostream>

namespace chrono {
namespace sensor {

__global__ void lidar_clip_kernel(float* buf, int w, int h, float threshold, float default_dist) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    if (out_index < w * h) {
        // data is packed range,intensity
        if (buf[2 * out_index + 1] < threshold) {
            buf[2 * out_index + 1] = 0;
            buf[2 * out_index] = default_dist;
        }
    }
}

void cuda_lidar_clip(float* buf, int width, int height, float threshold, float default_dist) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;
    lidar_clip_kernel<<<nBlocks, nThreads>>>(buf, width, height, threshold, default_dist);
}

}  // namespace sensor
}  // namespace chrono
