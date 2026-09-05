// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================
// GPU (Metal compute) implementation of the physics-based camera image
// pipeline. The Metal analogue of chrono_sensor/cuda/phys_cam_ops.cuh: same
// five stages, same parameters and units, ported kernel-for-kernel in
// shaders/ChMetalPhysCamOpsMSL.h.
//
// Unlike the CUDA entry points these take HOST pointers (the Metal sensor
// buffers live in host memory) and stage them through a shared MTLBuffer.
// Every call returns false if Metal is unavailable or the kernels failed to
// compile, in which case the caller must fall back to its CPU loop.
//
// Header is pure C++ (no Metal types); all Metal code lives in the .mm.
// =============================================================================

#ifndef CH_METAL_PHYS_CAM_OPS_H
#define CH_METAL_PHYS_CAM_OPS_H

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

/// GPU (Metal compute) ports of the phys_cam_ops.cu kernels.
/// All image buffers are host pointers to width*height RGBA float4 pixels
/// (Chrono's PixelHalf4 / PixelRGBDHalf4 on the non-OptiX builds).
namespace metal_phys_cam {

/// True if a Metal device is present and the phys-camera kernels compiled.
CH_SENSOR_API bool Available();

/// Depth-driven defocus blur. Port of cuda_phys_cam_defocus_blur.
/// @param buf_in  input RGB-D image (D = distance in the alpha slot), [m]
/// @param buf_out output RGBA image
/// @param f focal length, [m]
/// @param U focus distance, [m]
/// @param N aperture number = focal_length / aperture_diameter, [1/1]
/// @param C length of a pixel, [m]
CH_SENSOR_API bool
DefocusBlur(const void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h, float f, float U, float N, float C, float defocus_gain, float defocus_bias);

/// Illumination falloff toward the frame corners. Port of cuda_phys_cam_vignetting.
/// @param f focal length, [m]
/// @param L sensor width, [m]
/// @param G_vignet proportional gain of illumination falloff, [1/1]
CH_SENSOR_API bool Vignetting(void* buf_in_out, unsigned int img_w, unsigned int img_h, float f, float L, float G_vignet);

/// Integrate irradiance over exposure time and pixel area. Port of cuda_phys_cam_aggregator.
/// @param N aperture number, [1/1]
/// @param t exposure time, [sec]
/// @param C pixel size, [m]
/// @param P maximum scene light amount, [lux]
/// @param rgb_QEs 3 RGB quantum efficiencies, [1/1]
CH_SENSOR_API bool Aggregator(void* buf_in_out, unsigned int img_w, unsigned int img_h, float N, float t, float C, float P, const float* rgb_QEs, float G_aggregator);

/// Shot / dark-current / read / fixed-pattern noise. Port of cuda_phys_cam_noise.
/// curand's stream cannot be reproduced, so the two RNG streams are PCG-based:
/// the distributions match the CUDA model, individual samples do not.
/// @param t exposure time, [sec]
/// @param dark_currents 3 temporal dark currents, [electrons/sec]
/// @param noise_gains 3 temporal noise gains, [1/1]
/// @param sigma_reads 3 STDs of FPN and readout noise, [electrons]
/// @param shot_seed seed of the shot/dark noise stream
/// @param fpn_seed seed of the FPN/read noise stream
/// @param frame frame counter, decorrelates successive frames
CH_SENSOR_API bool Noise(void* buf_in_out,
                         unsigned int img_w,
                         unsigned int img_h,
                         float t,
                         const float* dark_currents,
                         const float* noise_gains,
                         const float* sigma_reads,
                         unsigned int shot_seed,
                         unsigned int fpn_seed,
                         unsigned int frame);

/// Exposure -> digital value through the camera response function.
/// Port of cuda_phys_cam_expsr2dv.
/// @param ISO analog amplification factor, [1/1]
/// @param gains 3 proportional gains, [1/1]
/// @param biases 3 biases, [1/1]
/// @param gamma gamma of the gamma-correction CRF
/// @param crf_type 0 gamma_correct, 1 sigmoid, 2 linear
CH_SENSOR_API bool
ExpsrToDV(const void* buf_in, void* buf_out, unsigned int img_w, unsigned int img_h, float ISO, const float* gains, const float* biases, float gamma, int crf_type);

}  // namespace metal_phys_cam

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
