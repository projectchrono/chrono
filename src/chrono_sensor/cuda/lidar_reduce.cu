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

// Converts 32bpp ARGB imgIn pixels to 8bpp Grayscale imgOut pixels
__global__ void mean_reduce_kernel(float* bufIn, float* bufOut, int w, int h, int r) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int out_hIndex = out_index % w;
    int out_vIndex = out_index / w;

    int d = r * 2 - 1;

    if (out_index < w * h) {
        // reset buffer to zeros
        bufOut[2 * out_index] = 0;
        bufOut[2 * out_index + 1] = 0;

        float sum_range = 0.f;
        float sum_intensity = 0.f;
        int n_contributing = 0;
        // gather up all of our values, take mean and push to output buffer
        for (int i = 0; i < d; i++) {
            for (int j = 0; j < d; j++) {
                int in_index = (d * out_vIndex + i) * d * w + (d * out_hIndex + j);
                sum_intensity += bufIn[2 * in_index + 1];
                if (bufIn[2 * in_index + 1] > 1e-6) {
                    sum_range += bufIn[2 * in_index];
                    n_contributing++;
                }
            }
        }
        if (n_contributing > 0) {
            bufOut[2 * out_index] = sum_range / (n_contributing);
            bufOut[2 * out_index + 1] = sum_intensity / (d * d);
        }
    }
}

// Converts 32bpp ARGB imgIn pixels to 8bpp Grayscale imgOut pixels
__global__ void strong_reduce_kernel(float* bufIn, float* bufOut, int w, int h, int r) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int out_hIndex = out_index % w;
    int out_vIndex = out_index / w;

    int d = r * 2 - 1;

    // float* raw_range = new float[d * d];
    // float* raw_intensity = new float[d * d];
    // int raw_id = 0;

    // extract the values we will use in our return distribution
    if (out_index < w * h) {
        float strongest = 0;
        float intensity_at_strongest = 0;

        // perform kernel operation to find max intensity
        float kernel_radius = .05;  // 10 cm total kernel width

        for (int i = 0; i < d; i++) {
            for (int j = 0; j < d; j++) {
                int in_index = (d * out_vIndex + i) * d * w + (d * out_hIndex + j);
                // float range = bufIn[2 * in_index];
                // float intensity = bufIn[2 * in_index + 1];

                float local_range = bufIn[2 * in_index];
                float local_intensity = bufIn[2 * in_index + 1];

                for (int k = 0; k < d; k++) {
                    for (int l = 0; l < d; l++) {
                        int inner_in_index = (d * out_vIndex + k) * d * w + (d * out_hIndex + l);
                        float range = bufIn[2 * inner_in_index];
                        float intensity = bufIn[2 * inner_in_index + 1];

                        if (inner_in_index != in_index && abs(range - local_range) < kernel_radius) {
                            float weight = (kernel_radius - abs(range - local_range)) / kernel_radius;
                            local_intensity += weight * intensity;
                            // norm_val += weight;
                        }
                    }
                }

                local_intensity = local_intensity / (d * d);  // calculating portion of beam here
                if (local_intensity > intensity_at_strongest) {
                    intensity_at_strongest = local_intensity;
                    strongest = local_range;
                }

            }
        }
        bufOut[2 * out_index] = strongest;
        bufOut[2 * out_index + 1] = intensity_at_strongest;
    }

}

__global__ void first_reduce_kernel(float* bufIn, float* bufOut, int w, int h, int r) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int out_hIndex = out_index % w;
    int out_vIndex = out_index / w;

    int d = r * 2 - 1;

    if (out_index < w * h) {
        float shortest = 1e10;
        float intensity_at_shortest = 0;

        // perform kernel operation to find max intensity
        float kernel_radius = .05;  // 10 cm total kernel width

        for (int i = 0; i < d; i++) {
            for (int j = 0; j < d; j++) {
                int in_index = (d * out_vIndex + i) * d * w + (d * out_hIndex + j);
                // float range = bufIn[2 * in_index];
                // float intensity = bufIn[2 * in_index + 1];

                float local_range = bufIn[2 * in_index];
                float local_intensity = bufIn[2 * in_index + 1];
                float ray_intensity = local_intensity;

                for (int k = 0; k < d; k++) {
                    for (int l = 0; l < d; l++) {
                        int inner_in_index = (d * out_vIndex + k) * d * w + (d * out_hIndex + l);
                        float range = bufIn[2 * inner_in_index];
                        float intensity = bufIn[2 * inner_in_index + 1];

                        if (inner_in_index != in_index && abs(range - local_range) < kernel_radius) {
                            float weight = (kernel_radius - abs(range - local_range)) / kernel_radius;
                            local_intensity += weight * intensity;
                        }
                    }
                }

                local_intensity = local_intensity / (d * d);  // calculating portion of beam here
                if (shortest > local_range && ray_intensity > 0) {
                    intensity_at_shortest = local_intensity;
                    shortest = local_range;
                }
            }
        }
        bufOut[2 * out_index] = shortest;
        bufOut[2 * out_index + 1] = intensity_at_shortest;
    }
}

__global__ void dual_reduce_kernel(float* bufIn, float* bufOut, int w, int h, int r) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int out_hIndex = out_index % w;
    int out_vIndex = out_index / w;

    int d = r * 2 - 1;

    if (out_index < w * h) {
        float shortest = 1e10;  // very very far
        float intensity_at_shortest = 0;
        float strongest = 0;
        float intensity_at_strongest = 0;

        // perform kernel operation to find max intensity
        float kernel_radius = .05;  // 10 cm total kernel width

        for (int i = 0; i < d; i++) {
            for (int j = 0; j < d; j++) {
                int in_index = (d * out_vIndex + i) * d * w + (d * out_hIndex + j);
                // float range = bufIn[2 * in_index];
                // float intensity = bufIn[2 * in_index + 1];

                float local_range = bufIn[2 * in_index];
                float local_intensity = bufIn[2 * in_index + 1];
                float ray_intensity = local_intensity;

                for (int k = 0; k < d; k++) {
                    for (int l = 0; l < d; l++) {
                        int inner_in_index = (d * out_vIndex + k) * d * w + (d * out_hIndex + l);
                        float range = bufIn[2 * inner_in_index];
                        float intensity = bufIn[2 * inner_in_index + 1];

                        if (inner_in_index != in_index && abs(range - local_range) < kernel_radius) {
                            float weight = (kernel_radius - abs(range - local_range)) / kernel_radius;
                            local_intensity += weight * intensity;
                        }
                    }
                }

                local_intensity = local_intensity / (d * d);  // calculating portion of beam here
                if (shortest > local_range && ray_intensity > 0) {
                    intensity_at_shortest = local_intensity;
                    shortest = local_range;
                }
                if (local_intensity > intensity_at_strongest) {
                    intensity_at_strongest = local_intensity;
                    strongest = local_range;
                }
            }
        }
        bufOut[4 * out_index] = strongest;
        bufOut[4 * out_index + 1] = intensity_at_strongest;
        bufOut[4 * out_index + 2] = shortest;
        bufOut[4 * out_index + 3] = intensity_at_shortest;
    }
}

void cuda_lidar_mean_reduce(void* bufIn, void* bufOut, int width, int height, int radius, CUstream& stream) {
    int w = width / (radius * 2 - 1);
    int h = height / (radius * 2 - 1);
    int numPixels = w * h;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;
    mean_reduce_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, w, h, radius);
}

void cuda_lidar_strong_reduce(void* bufIn, void* bufOut, int width, int height, int radius, CUstream& stream) {
    int w = width / (radius * 2 - 1);
    int h = height / (radius * 2 - 1);
    int numPixels = w * h;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;
    strong_reduce_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, w, h, radius);
}

void cuda_lidar_first_reduce(void* bufIn, void* bufOut, int width, int height, int radius, CUstream& stream) {
    int w = width / (radius * 2 - 1);
    int h = height / (radius * 2 - 1);
    int numPixels = w * h;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;
    first_reduce_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, w, h, radius);
}

void cuda_lidar_dual_reduce(void* bufIn, void* bufOut, int width, int height, int radius, CUstream& stream) {
    int w = width / (radius * 2 - 1);
    int h = height / (radius * 2 - 1);
    int numPixels = w * h;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;
    dual_reduce_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, w, h, radius);
}

}  // namespace sensor
}  // namespace chrono
