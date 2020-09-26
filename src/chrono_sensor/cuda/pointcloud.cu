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
#include "pointcloud.cuh"

namespace chrono {
namespace sensor {

// Converts 32bpp ARGB imgIn pixels to 8bpp Grayscale imgOut pixels
__global__ void pointcloud_from_depth_kernel(float* imgIn,
                                             float* imgOut,
                                             int w,
                                             int h,
                                             float hfov,
                                             float max_v_angle,
                                             float min_v_angle) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index < w * h) {
        int hIndex = index % w;
        int vIndex = index / w;

        float vAngle = (vIndex / (float)(h)) * (max_v_angle - min_v_angle) + min_v_angle;

        float hAngle = (hIndex / (float)(w)) * hfov - hfov / 2.;

        float range = imgIn[2 * index];

        float proj_xy = range * cos(vAngle);

        float x = proj_xy * cos(hAngle);
        float y = proj_xy * sin(hAngle);
        float z = range * sin(vAngle);
        imgOut[4 * index] = x;
        imgOut[4 * index + 1] = y;
        imgOut[4 * index + 2] = z;
        imgOut[4 * index + 3] = imgIn[2 * index + 1];
    }
}

void cuda_pointcloud_from_depth(void* bufDI,
                                void* bufOut,
                                int width,
                                int height,
                                float hfov,
                                float max_v_angle,
                                float min_v_angle) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;

    pointcloud_from_depth_kernel<<<nBlocks, nThreads>>>((float*)bufDI, (float*)bufOut, width, height, hfov, max_v_angle,
                                                        min_v_angle);
}

}  // namespace sensor
}  // namespace chrono
