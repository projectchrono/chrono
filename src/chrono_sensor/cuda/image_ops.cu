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
#include "image_ops.cuh"
#include "chrono_sensor/optix/shaders/device_utils.h"
#include <iostream>

namespace chrono {
namespace sensor {

__global__ void image_gauss_kernel_vert(unsigned char* buf, int w, int h, int c, int f_width, float* dweights) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    // only run for each output pixel
    if (index < w * h * c) {
        // float f_std = (float)f / 2.f;
        // int f_width = (int)(2.f * 3.14f * f_std);

        int channel = index % c;
        int col = index / c % w;
        int row = index / c / w;

        float sum = 0;
        for (int i = -f_width; i <= f_width; i++) {
            int index_in = channel + col * c + abs(row + i) * w * c;
            if (row + i >= h)
                index_in = channel + col * c + (2 * h - (row + i + 1)) * w * c;

            // float weight = exp(-i * i / (2 * f_std * f_std)) / sqrtf(2.f * 3.14f * f_std * f_std);
            sum += dweights[i + f_width] * ((float)buf[index_in]);
            // sum += ((float)buf[index_in]);
        }
        sum = fminf(255.f,fmaxf(0.f,sum));
        buf[index] = (unsigned char)(sum);
    }
}

__global__ void image_gauss_kernel_horiz(unsigned char* buf, int w, int h, int c, int f_width, float* dweights) {
    int index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    // only run for each output pixel
    if (index < w * h * c) {
        int channel = index % c;
        int col = index / c % w;
        int row = index / c / w;
        float sum = 0;
        for (int i = -f_width; i <= f_width; i++) {
            int index_in = channel + abs(col + i) * c + row * w * c;
            if (col + i >= w)
                index_in = channel + (2 * w - (col + i + 1)) * c + row * w * c;
            sum += dweights[i + f_width] * ((float)buf[index_in]);
        }
        sum = fminf(255.f,fmaxf(0.f,sum));
        buf[index] = (unsigned char)(sum);
    }
}

// merge pixels by the factor
__global__ void image_alias_kernel(unsigned char* bufIn,
                                   unsigned char* bufOut,
                                   int w_out,
                                   int h_out,
                                   int factor,
                                   int pix_size) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int w_in = w_out * factor;
    int h_in = h_out * factor;
    //
    // // only run for each output pixel
    if (out_index < w_out * h_out * pix_size) {
        int idc_out = out_index % pix_size;
        int idx_out = (out_index / pix_size) % w_out;
        int idy_out = (out_index / pix_size) / w_out;

        float mean = 0.0;

        for (int i = -1; i < factor + 1; i++) {
            for (int j = -1; j < factor + 1; j++) {
                int idc_in = idc_out;
                int idx_in = idx_out * factor + j;
                int idy_in = idy_out * factor + i;

                // reflect when out of range

                if (idx_in < 0)
                    idx_in = -idx_in - 1;
                else if (idx_in >= w_in)
                    idx_in = 2 * w_in - (idx_in + 1);
                if (idy_in < 0)
                    idy_in = -idy_in - 1;
                else if (idy_in >= h_in)
                    idy_in = 2 * h_in - (idy_in + 1);

                int in_index = idy_in * w_in * pix_size + idx_in * pix_size + idc_in;
                mean += (float)bufIn[in_index];
            }
        }
        // bufOut[out_index] = (unsigned char)(mean / (factor * factor));
        bufOut[out_index] = (unsigned char)(mean / ((factor + 2) * (factor + 2)));
        if (idc_out == 3) {
            bufOut[out_index] = 255;
        }
        // bufOut[out_index] = (unsigned char)(25 * idc_out);
    }
}

// merge pixels by the factor
__global__ void image_alias_float_kernel(float* bufIn, float* bufOut, int w_out, int h_out, int factor, int pix_size) {
    int out_index = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer

    int w_in = w_out * factor;
    //
    // // only run for each output pixel
    if (out_index < w_out * h_out * pix_size) {
        int idc_out = out_index % pix_size;
        int idx_out = (out_index / pix_size) % w_out;
        int idy_out = (out_index / pix_size) / w_out;

        float mean = 0.f;

        for (int i = 0; i < factor; i++) {
            for (int j = 0; j < factor; j++) {
                int idc_in = idc_out;
                int idx_in = idx_out * factor + j;
                int idy_in = idy_out * factor + i;

                int in_index = idy_in * w_in * pix_size + idx_in * pix_size + idc_in;
                mean += bufIn[in_index];
            }
        }
        bufOut[out_index] = mean / (factor * factor);
    }
}
// merge pixels by the factor
__global__ void image_half4_to_uchar4_kernel(__half* bufIn, unsigned char* bufOut, int N) {
    int idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (idx < N) {
        bufOut[idx] = (unsigned char)(clamp(__half2float(bufIn[idx]), 0.f, 1.f) * 255.f);
    }
}

void cuda_image_gauss_blur_char(void* buf, int w, int h, int c, int factor, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w * h * c + nThreads - 1) / nThreads;

    float f_std = (float)factor / 4.f;
    int f_width = (int)(3.14f * f_std);

    int entries = 2 * f_width + 1;

    float* weights = new float[entries];

    for (int i = 0; i <= 2 * f_width; i++) {
        int offset = i - f_width;
        weights[i] = exp(-offset * offset / (2 * f_std * f_std)) / sqrtf(2.f * 3.14f * f_std * f_std);
    }
    float* dweights;
    cudaMalloc(&dweights, entries * sizeof(float));
    cudaMemcpy(dweights, weights, entries * sizeof(float), cudaMemcpyHostToDevice);

    image_gauss_kernel_vert<<<nBlocks, nThreads, 0, stream>>>((unsigned char*)buf, w, h, c, f_width, dweights);
    image_gauss_kernel_horiz<<<nBlocks, nThreads, 0, stream>>>((unsigned char*)buf, w, h, c, f_width, dweights);
    cudaFree(dweights);
    delete[] weights;
}

void cuda_image_alias(void* bufIn, void* bufOut, int w_out, int h_out, int factor, int pix_size, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w_out * h_out * pix_size + nThreads - 1) / nThreads;

    image_alias_kernel<<<nBlocks, nThreads, 0, stream>>>((unsigned char*)bufIn, (unsigned char*)bufOut, w_out, h_out,
                                                         factor, pix_size);
}

void cuda_image_alias_float(void* bufIn,
                            void* bufOut,
                            int w_out,
                            int h_out,
                            int factor,
                            int pix_size,
                            CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w_out * h_out * pix_size + nThreads - 1) / nThreads;

    image_alias_float_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, w_out, h_out, factor,
                                                               pix_size);
}

void cuda_image_half4_to_uchar4(void* bufIn, void* bufOut, int w, int h, CUstream& stream) {
    const int nThreads = 512;
    int nBlocks = (w * h * 4 + nThreads - 1) / nThreads;
    image_half4_to_uchar4_kernel<<<nBlocks, nThreads, 0, stream>>>((__half*)bufIn, (unsigned char*)bufOut, w * h * 4);
}

}  // namespace sensor
}  // namespace chrono
