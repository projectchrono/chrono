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

namespace chrono {
namespace sensor {

// Converts 32bpp ARGB imgIn pixels to 8bpp Grayscale imgOut pixels
__global__ void grayscale_kernel(int* imgIn, char* imgOut, int numPixels) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index < numPixels) {
        int v = imgIn[index];  // ARGB
        int r = v >> 16 & 0xFF;
        int b = v >> 8 & 0xFF;
        int g = v >> 0 & 0xFF;
        int gs = (r + b + g) / 3;  // simple average
        imgOut[index] = (char)gs;
    }
}

void cuda_grayscale(void* bufRGBA, void* bufOut, int width, int height) {
    int numPixels = width * height;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;

    grayscale_kernel<<<nBlocks, nThreads>>>((int*)bufRGBA, (char*)bufOut, numPixels);
}

}  // namespace sensor
}  // namespace chrono
