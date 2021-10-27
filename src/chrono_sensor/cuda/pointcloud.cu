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
#include <iostream>

namespace chrono {
namespace sensor {

// Converts a depth and intensity buffer to an XZY and intensity buffer
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
        float proj_xy = range * cosf(vAngle);
        float x = proj_xy * cosf(hAngle);
        float y = proj_xy * sinf(hAngle);
        float z = range * sinf(vAngle);
        imgOut[4 * index] = x;
        imgOut[4 * index + 1] = y;
        imgOut[4 * index + 2] = z;
        imgOut[4 * index + 3] = imgIn[2 * index + 1];
//        printf("%f %f %f\n", x, y, z);
    }
}
// Converts a depth and intensity buffer to an XZY and intensity buffer
__global__ void pointcloud_from_depth_dual_kernel(float* imgIn,
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

        float strongest_range = imgIn[4 * index];
        float strongest_proj_xy = strongest_range * cos(vAngle);
        float strongest_x = strongest_proj_xy * cos(hAngle);
        float strongest_y = strongest_proj_xy * sin(hAngle);
        float strongest_z = strongest_range * sin(vAngle);

        float shortest_range = imgIn[4 * index + 2];
        float shortest_proj_xy = shortest_range * cos(vAngle);
        float shortest_x = shortest_proj_xy * cos(hAngle);
        float shortest_y = shortest_proj_xy * sin(hAngle);
        float shortest_z = shortest_range * sin(vAngle);

        imgOut[8 * index] = strongest_x;
        imgOut[8 * index + 1] = strongest_y;
        imgOut[8 * index + 2] = strongest_z;
        imgOut[8 * index + 3] = imgIn[4 * index + 1];
        imgOut[8 * index + 4] = shortest_x;
        imgOut[8 * index + 5] = shortest_y;
        imgOut[8 * index + 6] = shortest_z;
        imgOut[8 * index + 7] = imgIn[4 * index + 3];
    }
}





void cuda_pointcloud_from_depth_dual_return(void* bufDI,
                                            void* bufOut,
                                            int width,
                                            int height,
                                            float hfov,
                                            float max_v_angle,
                                            float min_v_angle,
                                            CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;

    pointcloud_from_depth_dual_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufDI, (float*)bufOut, width, height,
                                                                        hfov, max_v_angle, min_v_angle);
}

void cuda_pointcloud_from_depth(void* bufDI,
                                void* bufOut,
                                int width,
                                int height,
                                float hfov,
                                float max_v_angle,
                                float min_v_angle,
                                CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;
    pointcloud_from_depth_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufDI, (float*)bufOut, width, height, hfov,
                                                                   max_v_angle, min_v_angle);
}


}  // namespace sensor
}  // namespace chrono
