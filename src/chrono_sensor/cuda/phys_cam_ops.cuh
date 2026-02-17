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
// util cuda functions for filters in the physics-based camera 
//
// =============================================================================
#include <cuda_fp16.h>
// #include <string>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_cuda
/// @{

/// host function for applying defocus blur to an image based on the depth map
/// @param buf_in  A device pointer to the input RGBD image.
/// @param buf_out A device pointer to the ouput RGBA image.
/// @param img_w The input/output image width, [px]
/// @param img_h The input/output image height, [px]
/// @param f focal length, [m]
/// @param U focus distance, [m]
/// @param N F-number (or aperture number) = focal_length / aperture_diameter, [1/1]
/// @param C length of a pixel, [m]
/// @param G_defocus proportional gain of defocus blur diameter, [1/1]
/// @param stream cuda stream for computation
void cuda_phys_cam_defocus_blur(
    void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h, float f, float U, float N, float C,
    float defocus_gain, float defocus_bias, CUstream& stream
);



/// host function for applying vignetting to an image based on camera control and model parameters
/// @param buf_in_out  A device pointer to the input/output RGBA(half4) image.
/// @param img_w The input/output image width, [px]
/// @param img_h The input/output image height, [px]
/// @param f focal length, [m]
/// @param L sensor width, [m]
/// @param G_vignet proportional gain of illumination falloff, [1/1]
/// @param stream cuda stream for computation
void cuda_phys_cam_vignetting(
    void* buf_in_out, unsigned int img_w, unsigned int img_h, float f, float L, float G_vignet, CUstream& stream
);

/// host function for applying amplification to an image, based on camera control and model parameters
/// @param buf_in_out  A device pointer to the input/output RGBA(half4) image.
/// @param img_w The input/output image width, [px]
/// @param img_h The input/output image height, [px]
/// @param N aperture number = focal_length / aperture_diameter, [1/1]
/// @param t exposure time, [sec]
/// @param C pixel size, [m]
/// @param P maximum scene light amount, [lux = lm/m^2]
/// @param host_rgb_QEs array of RGB quantum efficiencies, [1/1]
/// @param G_aggregator proportional gain of illumination aggregation, [1/1]
/// @param stream CUDA stream for computation
void cuda_phys_cam_aggregator(
    void* buf_in_out, unsigned int img_w, unsigned int img_h, float N, float t, float C, float P, float* host_rgb_QEs,
    float G_aggregator, CUstream& stream
);

/// host function for applying noise model to the image, based on camera control and model parameters
/// @param buf_in_out  A device pointer to the input/output RGBA(half4) image.
/// @param img_w The input/output image width, [px]
/// @param img_h The input/output image height, [px]
/// @param t exposure time, [sec]
/// @param host_dark_cuurents array of temporal dark currents and hot pixels, [electrons/sec]
/// @param host_noise_gains array of noise gains for temporal noises, [1/1]
/// @param host_sigma_reads array of STDs of FPN and readout noises, [electrons]
/// @param rng_shot random number generator of shot, dark, and hot-pixel noises
/// @param rng_FPN random number generator of FPN and read noises
/// @param stream CUDA stream for computation
void cuda_phys_cam_noise(
    void* buf_in_out, unsigned int img_w, unsigned int img_h, float t, float* host_dark_cuurents,
    float* host_noise_gains, float* host_sigma_reads, curandState_t* rng_shot, curandState_t* rng_FPN, CUstream& stream
);

/// host function for converting exposure to digital values
/// @param buf_in  A device pointer to the input RGBA(Half4) buffer.
/// @param buf_out A device pointer to the ouput RGBA16 buffer.
/// @param img_w The input/output image width, [px]
/// @param img_h The input/output image height, [px]
/// @param ISO (ISO) analog amplification factor, [1/1]
/// @param host_gains array of proportional gains, [1/1]
/// @param host_biases array of biases or intercepts, [1/1]
/// @param gamma gamma of the gamma correction function if used, [DV]
/// @param crf_type type of the camera response function (CRF), 0: "gamma_correct", 1: "sigmoid"
/// @param stream CUDA stream for computation
void cuda_phys_cam_expsr2dv(
    void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h, float ISO, float* host_gains, float* host_biases,
    float gamma, int crf_type, CUstream& stream
);

/// @}

}  // namespace sensor
}  // namespace chrono