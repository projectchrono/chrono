// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bo-Hsun Chen
// =============================================================================
//
// =============================================================================
#define _USE_MATH_DEFINES
#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>
#include <cuda_runtime.h>

#include "curand_utils.cuh"
#include "phys_cam_ops.cuh"
#include "chrono_sensor/optix/shaders/device_utils.h"
#include <iostream>
#include <math.h>

namespace chrono {
namespace sensor {

#define WAVELENGTH_RED 700.0 	// [nm]
#define WAVELENGTH_GREEN 546.1 	// [nm]
#define WAVELENGTH_BLUE 435.8 	// [nm]


////---- Functions for cuda_phys_cam_defocus_blur ----////

/// Gaussian 1D discrete function
/// @param kernel_size Gaussian kernel size, must be odd and positive
/// @param sigma standard deviation of Gaussian function
inline __device__ __half Gaussian1D(int x, int kernel_size, float sigma) {
    float coeff = 1.0f / (sqrtf(2.0f * M_PI) * sigma);
    float exponent = - (float)(x * x) / (2.0f * sigma * sigma);
    return __float2half(coeff * expf(exponent));
}

// kernel function
__global__ void cuda_phys_cam_defocus_blur_kernel(__half* buf_in, __half* buf_out, unsigned int img_w, unsigned int img_h,
                                                  float f, float U, float N, float C, float defocus_gain, float defocus_bias) {
    int pixel_num = img_w * img_h;
    int px_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index into output buffer
    if (px_idx < pixel_num) {
        
        // initialize output buffer values
        for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
            buf_out[4 * px_idx + ch_idx] = 0.;
        }
        __syncthreads();
        
        int kernel_size = 0; // defocus blur diameter, [px]
        float d = __half2float(buf_in[px_idx * 4 + 3]); // distance, [m]
        
        // distance larger than 0, calculate defocus blur diameter
        if (d > 1e-9) {
            kernel_size = int(ceilf(f * f * fabsf(d - U) / (N * C * d * (U - f))));
        }
        
        if (kernel_size > 1) {
            //---- apply Gaussian blur ----//
            kernel_size = max(1, int(defocus_gain * kernel_size + defocus_bias));

            // ensure kernel size is odd
            kernel_size = (kernel_size % 2 == 1) ? (kernel_size) : (kernel_size + 1);
            // printf("%d\n", kernel_size);

            int x_src = px_idx % img_w;
            int y_src = px_idx / img_w;
            
            // do multiplication for each pixel
            int x_min = max(0, x_src - (kernel_size - 1) / 2);
            int x_max = min(img_w, x_src + (kernel_size - 1) / 2);
            int y_min = max(0, y_src - (kernel_size - 1) / 2);
            int y_max = min(img_h, y_src + (kernel_size - 1) / 2);
            
            for (int y = y_min; y <= y_max; ++y) {
                for (int x = x_min; x <= x_max; ++x) {
                    // iterate over R, G, and B channels
                    for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
                        // atomicAdd(
                        //     buf_out + 4 * (y * img_w + x) + ch_idx, buf_in[4 * px_idx + ch_idx] *
                        //     Gaussian1D(y - y_src, kernel_size, D / 6) * Gaussian1D(x - x_src, kernel_size, D / 6)
                        // );
                        buf_out[4 * px_idx + ch_idx] += buf_in[4 * (y * img_w + x) + ch_idx]
                            * Gaussian1D(y - y_src, kernel_size, float(kernel_size) / 6.f)
                            * Gaussian1D(x - x_src, kernel_size, float(kernel_size) / 6.f);
                    }
                }
            }
        }
        else {
            for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
                // atomicAdd(buf_out + 4 * px_idx + ch_idx, buf_in[4 * px_idx + ch_idx]);
                buf_out[4 * px_idx + ch_idx] = buf_in[4 * px_idx + ch_idx];
            }
        }
        __syncthreads();

        // set A channel to 1.0
        buf_out[4 * px_idx + 3] = 1.0;
    }
}

// host function
__host__ void cuda_phys_cam_defocus_blur(void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h, float f,
                                         float U, float N, float C, float defocus_gain, float defocus_bias,
                                         CUstream& stream) {
    // Set up kernel launch configuration
    const int threads_per_block = 512;
    const int blocks_per_grid = (img_w * img_h + threads_per_block - 1) / threads_per_block;
    
    // Launch the kernel
    cuda_phys_cam_defocus_blur_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
        (__half*)buf_in, (__half*)buf_out, img_w, img_h, f, U, N, C, defocus_gain, defocus_bias
    );
    cudaDeviceSynchronize();
}


////---- Functions for cuda_phys_cam_vignetting ----////

// kernel function
__global__ void cuda_phys_cam_vignetting_kernel(__half* buf_in_out, unsigned int img_w, unsigned int img_h,
                                                  float f, float L, float G_vignet) {
    
    int pixel_num = img_w * img_h;
    int px_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // index in output buffer
    if (px_idx < pixel_num) {
        // get pixel coordinates
        int col = px_idx % img_w;
        int row = px_idx / img_w;

        // get equivalent length coordinates and radius
        float x = ((float)col - (float)(img_w - 1) / 2.f) / (float)(img_w) * L;
        float y = ((float)row - (float)(img_h - 1) / 2.f) / (float)(img_w) * L;
        
        // E <- E * (1 - G_vignet + G_vignet * (cos(theta))^4)
        for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
            buf_in_out[4 * px_idx + ch_idx] *= (__half)(1.f - G_vignet + G_vignet * pow(cosf(atanf(sqrtf(x * x + y * y) / f)), 4));
        }
    }
}

// host function
__host__ void cuda_phys_cam_vignetting(void* buf_in_out, unsigned int img_w, unsigned int img_h, float f, float L, 
                                       float G_vignet, CUstream& stream) {
    // Set up kernel launch configuration
    const int threads_per_block = 512;
    const int blocks_per_grid = (img_w * img_h + threads_per_block - 1) / threads_per_block;

    // Launch the kernel
    cuda_phys_cam_vignetting_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
        (__half*)buf_in_out, img_w, img_h, f, L, G_vignet
    );
    cudaDeviceSynchronize();
}



////---- Functions for cuda_phys_cam_aggregator ----////

// kernel function
__global__ void cuda_phys_cam_aggregator_kernel(__half* buf_in_out, unsigned int img_w, unsigned int img_h,
                                                float N, float t, float C, float P, float *rgb_QEs, float G_aggregator) {
    int pixel_num = img_w * img_h;
    int px_idx = (blockDim.x * blockIdx.x + threadIdx.x); // pixel index in output buffer
    if (px_idx < pixel_num) {
        for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
            // [W/m^2] x [sec] x [m^2] = [J]
            buf_in_out[4 * px_idx + ch_idx] *= G_aggregator * P / N / N * C * C * t * rgb_QEs[ch_idx];
        }
    }
}

// host function
__host__ void cuda_phys_cam_aggregator(void* buf_in_out, unsigned int img_w, unsigned int img_h, float N, float t,
                                       float C, float P, float *host_rgb_QEs, float G_aggregator, CUstream& stream) {
    // Set up kernel launch configuration
    const int threads_per_block = 512;
    const int blocks_per_grid = (img_w * img_h + threads_per_block - 1) / threads_per_block;

    // Prepare arrays that are allocated as device memory
    float *dev_rgb_QEs;
    cudaMalloc((void**)&dev_rgb_QEs, sizeof(float) * 3);
    cudaMemcpy(dev_rgb_QEs, host_rgb_QEs, sizeof(float) * 3, cudaMemcpyHostToDevice);

    // Launch the kernel
    cuda_phys_cam_aggregator_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
        (__half*)buf_in_out, img_w, img_h, N, t, C, P, dev_rgb_QEs, G_aggregator
    );
    cudaDeviceSynchronize();
    cudaFree(dev_rgb_QEs);
}


////---- Functions for cuda_phys_cam_noise ----////

// kernel function
__global__ void cuda_phys_cam_noise_kernel(
    __half* buf_in_out, unsigned int img_w, unsigned int img_h, float t, float* dark_currents, float* noise_gains,
    float* sigma_reads, curandState_t* rng_shot, curandState_t* rng_FPN
) {
    int px_idx = (blockDim.x * blockIdx.x + threadIdx.x); // pixel index in output buffer
    if (px_idx < img_w * img_h) {
        
        // I = L + N_read
        // L ~ Poisson(\mu + D * t), 
        // calculate mean number of electrons in the pixel channel transducer
        // E = h * c / wavelength, number_of_electrons = Energy / E = Energy * wavelength / (h *c)
        // and 1 / (h * c) = 5.0341e24 [1/J-m]
        
        // if ((double)(buf_in_out[4 * px_idx + 2]) > 1e-12) printf("%f\n", __half2float(buf_in_out[4 * px_idx + 2])); // debug
        for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
            double e_num = (double)(buf_in_out[4 * px_idx + ch_idx]) + dark_currents[ch_idx] * t; // mean number of electrons

            // sample random variables L ~ Poisson(e_num + D * t)
            // if ((double)(buf_in_out[4 * px_idx + 0]) > 1e-12) printf("%f\n", ceil((e_num_R + e_num_dark)));
            e_num += curand_normal(&rng_shot[px_idx]) * noise_gains[ch_idx] * sqrt(e_num);

            // sample random variables N_read ~ Gaussian(0, sigma_read)
            e_num += curand_normal(&rng_FPN[px_idx]) * sigma_reads[ch_idx];

            // printf("gain: %f, sigma_read: %f\n", noise_gains[ch_idx], sigma_reads[ch_idx]);
            // if ((double)(buf_in_out[4 * px_idx + 0]) > 1e-12) printf("%f\n", e_num_R);

            // e_num_R = (double)( curand_poisson(&rng_shot[px_idx], 100.0 * (e_num_R + e_num_dark)) ) / 100.0;
            // e_num_G = (double)( curand_poisson(&rng_shot[px_idx], 100.0 * (e_num_G + e_num_dark)) ) / 100.0;
            // e_num_B = (double)( curand_poisson(&rng_shot[px_idx], 100.0 * (e_num_B + e_num_dark)) ) / 100.0;

            buf_in_out[4 * px_idx + ch_idx] = (__half)(e_num);
        }
    }
}

// host function
__host__ void cuda_phys_cam_noise(void* buf_in_out, unsigned int img_w, unsigned int img_h, float t, float* host_dark_currents,
                                  float* host_noise_gains, float* host_sigma_reads, curandState_t* rng_shot,
                                  curandState_t* rng_FPN, CUstream& stream) {
    // Set up kernel launch configuration
    const int threads_per_block = 512;
    const int blocks_per_grid = (img_w * img_h + threads_per_block - 1) / threads_per_block;

    // Prepare device-allocated arrays
    float *dev_dark_currents, *dev_noise_gains, *dev_sigma_reads;
    cudaMalloc((void**)&dev_dark_currents, sizeof(float) * 3);
    cudaMalloc((void**)&dev_noise_gains, sizeof(float) * 3);
    cudaMalloc((void**)&dev_sigma_reads, sizeof(float) * 3);
    cudaMemcpy(dev_dark_currents, host_dark_currents, sizeof(float) * 3, cudaMemcpyHostToDevice);
    cudaMemcpy(dev_noise_gains, host_noise_gains, sizeof(float) * 3, cudaMemcpyHostToDevice);
    cudaMemcpy(dev_sigma_reads, host_sigma_reads, sizeof(float) * 3, cudaMemcpyHostToDevice);

    // Launch the kernel
    cuda_phys_cam_noise_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
        (__half*)buf_in_out, img_w, img_h, t, dev_dark_currents, dev_noise_gains, dev_sigma_reads, rng_shot, rng_FPN
    );
    cudaDeviceSynchronize();

    cudaFree(dev_dark_currents);
    cudaFree(dev_noise_gains);

}


////---- Functions of cuda_phys_cam_expsr2dv ----////

// kernel function for linear function
__global__ void cuda_phys_cam_expsr2dv_kernel_linear(__half* buf_in, __half* buf_out, unsigned int img_w, unsigned int img_h,
                                                     float ISO, float* gains, float* biases) {
    
    int pixel_num = img_w * img_h;
    int px_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // pixel index in input/output buffer
    if (px_idx < pixel_num) {
        // convert from exposure domain to DV domain
        // I = gain * E + bias, I: [DV], from 0 to 1.0, E: exposure [electrons]
        for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
            float px_I = gains[ch_idx] * ISO * (float)(buf_in[4 * px_idx + ch_idx]) + biases[ch_idx];

            // buf_out[4 * px_idx + ch_idx] = (uint16_t)(clamp(px_I, 0.f, 1.f) * 65534.999f);
            buf_out[4 * px_idx + ch_idx] = (__half)px_I;

            // if ((double)(buf_in[4 * px_idx + ch_idx]) > 1e-12 && ch_idx == 0)
                // printf("%u\n", buf_out[4 * px_idx + ch_idx]);
                // printf("%f\n", px_I);
                // printf("%f\n", __half2float(buf_out[4 * px_idx + ch_idx]));
        }
        // buf_out[4 * px_idx + 3] = (uint16_t)(clamp(__half2float(buf_in[4 * px_idx + 3]), 0.f, 1.f) * 65534.999f);
        buf_out[4 * px_idx + 3] = buf_in[4 * px_idx + 3];
    }
}

// kernel function for sigmoid function (especially for film sensor)
__global__ void cuda_phys_cam_expsr2dv_kernel_sigmoid(__half* buf_in, __half* buf_out, unsigned int img_w, unsigned int img_h,
                                                      float ISO, float* gains, float* biases) {
    
    int pixel_num = img_w * img_h;
    int px_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // pixel index in input/output buffer
    if (px_idx < pixel_num) {
        // convert from exposure domain to DV domain
        // I = 1 / (1 + exp(-gain * lg(E) - bias)), I: [DV], from 0 to 1.0, E: exposure [electrons]
        for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
            float px_I = gains[ch_idx] * ISO * (float)(buf_in[4 * px_idx + ch_idx]) + biases[ch_idx];
            px_I = 1.f / (1.f + expf(-px_I));

            // buf_out[4 * px_idx + ch_idx] = (uint16_t)(clamp(px_I, 0.f, 1.f) * 65534.999f);
            buf_out[4 * px_idx + ch_idx] = (__half)px_I;
            // buf_out[4 * px_idx + ch_idx] = (uint16_t)(32783);

            // if ((double)(buf_in[4 * px_idx + ch_idx]) > 1e-12 && ch_idx == 0)
                // printf("%u\n", buf_out[4 * px_idx + ch_idx]);
                // printf("%f\n", px_I);
                // printf("%f\n", __half2float(buf_out[4 * px_idx + ch_idx]));
        }
        // buf_out[4 * px_idx + 3] = (uint16_t)(clamp(__half2float(buf_in[4 * px_idx + 3]), 0.f, 1.f) * 65534.999f);
        // buf_out[4 * px_idx + 3] = buf_in[4 * px_idx + 3];
        buf_out[4 * px_idx + 3] = 1.0;
    }
}

// kernel function for gamma_correct (especially for digital imaging sensor)
__global__ void cuda_phys_cam_expsr2dv_kernel_gamma(__half* buf_in, __half* buf_out, unsigned int img_w, unsigned int img_h,
                                                    float ISO, float* gains, float* biases, float gamma) {
    
    int pixel_num = img_w * img_h;
    int px_idx = (blockDim.x * blockIdx.x + threadIdx.x);  // pixel index in input/output buffer
    if (px_idx < pixel_num) {
        // convert from exposure domain to DV domain
        // I = a * (lg(E))^gamma + b, I: [DV] from 0 to 1.0, E: exposure [electrons]
        for (int ch_idx = 0; ch_idx < 3; ++ch_idx) {
            float px_I = gains[ch_idx] * powf(log2f(ISO * (float)(buf_in[4 * px_idx + ch_idx])), gamma) + biases[ch_idx];
            
            // buf_out[4 * px_idx + ch_idx] = (uint16_t)(clamp(px_I, 0.f, 1.f) * 65534.999f);
            buf_out[4 * px_idx + ch_idx] = (__half)px_I;

            // if ((double)(buf_in[4 * px_idx + ch_idx]) > 1e-12 && ch_idx == 0)
                // printf("%u\n", buf_out[4 * px_idx + ch_idx]);
                // printf("%f\n", px_I);
                // printf("%f\n", __half2float(buf_out[4 * px_idx + ch_idx]));
        }
        // buf_out[4 * px_idx + 3] = (uint16_t)(clamp(__half2float(buf_in[4 * px_idx + 3]), 0.f, 1.f) * 65534.999f);
        buf_out[4 * px_idx + 3] = buf_in[4 * px_idx + 3];
    }
}

// host function
__host__ void cuda_phys_cam_expsr2dv(void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h,
                                     float ISO, float* host_gains, float* host_biases, float gamma, int crf_type,
                                     CUstream& stream) {
    // Set up kernel launch configuration
    const int threads_per_block = 512;
    const int blocks_per_grid = (img_w * img_h + threads_per_block - 1) / threads_per_block;

    // Prepare arrays that are allocated as device memory
    float *dev_gains, *dev_biases;
    cudaMalloc((void**)&dev_gains, sizeof(float) * 3);
    cudaMalloc((void**)&dev_biases, sizeof(float) * 3);
    cudaMemcpy(dev_gains, host_gains, sizeof(float) * 3, cudaMemcpyHostToDevice);
    cudaMemcpy(dev_biases, host_biases, sizeof(float) * 3, cudaMemcpyHostToDevice);

    // Launch the kernel
    if (crf_type == 0) { // gamma correction function
        cuda_phys_cam_expsr2dv_kernel_gamma<<<blocks_per_grid, threads_per_block, 0, stream>>>(
            (__half*)buf_in, (__half*)buf_out, img_w, img_h, ISO, dev_gains, dev_biases, gamma
        );
        cudaDeviceSynchronize();
    }
    else if (crf_type == 1) { // sigmoid function
        cuda_phys_cam_expsr2dv_kernel_sigmoid<<<blocks_per_grid, threads_per_block, 0, stream>>>(
            (__half*)buf_in, (__half*)buf_out, img_w, img_h, ISO, dev_gains, dev_biases
        );
        cudaDeviceSynchronize();
    }
    else if (crf_type == 2) { // linear function
        cuda_phys_cam_expsr2dv_kernel_linear<<<blocks_per_grid, threads_per_block, 0, stream>>>(
            (__half*)buf_in, (__half*)buf_out, img_w, img_h, ISO, dev_gains, dev_biases
        );
        cudaDeviceSynchronize();
    }
    else {
        throw std::runtime_error("Invalid camera response function type");
    }

    cudaFree(dev_gains);
    cudaFree(dev_biases);
}

}  // namespace sensor
}  // namespace chrono