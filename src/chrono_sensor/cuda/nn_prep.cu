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
#include "nn_prep.cuh"
#include <iostream>

namespace chrono {
namespace sensor {

__global__ void RGBA8_to_FLOAT3_kernel(uint8_t* bufIn, float* bufOut, int h, int w) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    if (index < 3 * w * h) {
        int channel = index % 3;
        int col = index / 3 % w;
        int row = index / 3 / w;

        int index_in = channel + col * 4 + row * w * 4;

        bufOut[index] = ((float)bufIn[index_in]);
    }
}

// converts chars [0,255] to floats [-1,1]
__global__ void RGBA8_to_FLOAT4_kernel(uint8_t* bufIn, float* bufOut, int N) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    if (index < N) {
        bufOut[index] = ((float)bufIn[index]) / 127.5 - 1;
        // bufOut[index] = ((float)bufIn[index]) / 255.0;
        if (bufOut[index] > 1 || bufOut[index] < -1) {
            printf("mapped out of range\n");
        }
    }
}

// converts floats [-1,1] to chars [0,255]
__global__ void FLOAT4_to_RGBA8_kernel(float* bufIn, uint8_t* bufOut, int N) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    if (index < N) {
        bufOut[index] = (uint8_t)((bufIn[index] + 1.0) * 127.99);
    }
}

// converts chars [0,255] to floats [-1,1]
__global__ void RGBA8_to_FLOAT4_CHW_kernel(uint8_t* bufIn, float* bufOut, int c, int h, int w) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    if (index < c * h * w) {
        int channel = index % c;
        int col = index / c % w;
        int row = index / c / w;
        int index_out = col + row * w + channel * w * h;
        bufOut[index_out] = ((float)bufIn[index]) / 127.5 - 1;
    }
}

// converts chars [0,255] to floats [-1,1]
__global__ void normalize_float_kernel(float* buf, float add, float mult, int c, int h, int w) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    if (index < c * h * w) {
        buf[index] = (buf[index] + add) * mult;
    }
}

// converts floats [-1,1] to chars [0,255]
__global__ void FLOAT4_to_RGBA8_CHW_kernel(float* bufIn, uint8_t* bufOut, int c, int h, int w) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    if (index < c * h * w) {
        int channel = index % c;
        int col = index / c % w;
        int row = index / c / w;
        int index_out = col + row * w + channel * w * h;
        bufOut[index] = (uint8_t)((bufIn[index_out] + 1.0) * 127.99);
    }
}

void preprocess_RGBA8_to_FLOAT3(void* bufIn, void* bufOut, int h, int w) {
    const int nThreads = 512;
    int nBlocks = (w * h * 3 + nThreads - 1) / nThreads;

    RGBA8_to_FLOAT3_kernel<<<nBlocks, nThreads>>>((uint8_t*)bufIn, (float*)bufOut, h, w);
}

void preprocess_RGBA8_to_FLOAT4(void* bufIn, void* bufOut, int num_entries) {
    const int nThreads = 512;
    int nBlocks = (num_entries + nThreads - 1) / nThreads;

    RGBA8_to_FLOAT4_kernel<<<nBlocks, nThreads>>>((uint8_t*)bufIn, (float*)bufOut, num_entries);
}

void postprocess_FLOAT4_to_RGBA8(void* bufIn, void* bufOut, int num_entries) {
    const int nThreads = 512;
    int nBlocks = (num_entries + nThreads - 1) / nThreads;

    FLOAT4_to_RGBA8_kernel<<<nBlocks, nThreads>>>((float*)bufIn, (uint8_t*)bufOut, num_entries);
}

void preprocess_RGBA8_to_FLOAT4_CHW(void* bufIn, void* bufOut, int c, int h, int w) {
    const int nThreads = 512;
    int nBlocks = (c * h * w + nThreads - 1) / nThreads;

    RGBA8_to_FLOAT4_CHW_kernel<<<nBlocks, nThreads>>>((uint8_t*)bufIn, (float*)bufOut, c, h, w);
}

void preprocess_normalize_float(void* buf, float add, float mult, int c, int h, int w) {
    const int nThreads = 512;
    int nBlocks = (c * h * w + nThreads - 1) / nThreads;

    normalize_float_kernel<<<nBlocks, nThreads>>>((float*)buf, add, mult, c, h, w);
}

void postprocess_FLOAT4_to_RGBA8_CHW(void* bufIn, void* bufOut, int c, int h, int w) {
    const int nThreads = 512;
    int nBlocks = (c * h * w + nThreads - 1) / nThreads;

    FLOAT4_to_RGBA8_CHW_kernel<<<nBlocks, nThreads>>>((float*)bufIn, (uint8_t*)bufOut, c, h, w);
}

}  // namespace sensor
}  // namespace chrono
