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
// Metal Shading Language compute kernels for the physics-based camera image
// pipeline. Direct port of chrono_sensor/cuda/phys_cam_ops.cu -- see
// phys_cam_ops.cuh for the physical meaning and units of every parameter.
//
// The CUDA kernels operate on __half4 pixels; on the non-OptiX builds the
// Chrono PixelHalf4 / PixelRGBDHalf4 structs are four 32-bit floats, so these
// kernels use float4 and carry the arithmetic at full float precision (the
// CUDA version rounds to half on every store).
//
// Deliberate deviations from phys_cam_ops.cu, all of them bug fixes that the
// existing host-side fallback in ChFilterPhysCamera*.cpp already makes:
//   * defocus blur clamps the gather window to img_w-1 / img_h-1. The CUDA
//     kernel uses min(img_w, ...) and then loops x <= x_max, so it reads one
//     pixel past the end of each row (the next row's first pixel, or past the
//     end of the buffer on the last row).
//   * defocus blur guards the blur-diameter denominator against zero and takes
//     the magnitude of the result, so focus_dist <= focal_length cannot produce
//     a negative kernel size.
//   * the shot-noise term uses sqrt(max(0, e_num)); the CUDA version can take
//     the square root of a negative mean and produce NaN.
// =============================================================================

#ifndef CH_METAL_PHYS_CAM_OPS_MSL_H
#define CH_METAL_PHYS_CAM_OPS_MSL_H

static const char* kPhysCamOpsMSL = R"MSLGEN(
#include <metal_stdlib>
using namespace metal;

struct PhysCamParams {
    float4 qe;            // RGB quantum efficiencies
    float4 darkCurrent;   // [electrons/sec]
    float4 noiseGain;     // [1/1]
    float4 sigmaRead;     // [electrons]
    float4 gains;         // expsr2dv proportional gains
    float4 biases;        // expsr2dv biases
    uint  width, height;
    float f, U, N, C;     // focal length, focus distance, aperture number, pixel size
    float defocusGain, defocusBias;
    float L, GVignet;     // sensor width, vignetting gain
    float t, P, GAggregator;  // exposure time, max scene light amount, aggregator gain
    float ISO, gamma;
    uint  shotSeed, fpnSeed, frame;
};

// ---- Gaussian 1D discrete function (phys_cam_ops.cu: Gaussian1D) ----
static inline float Gaussian1D(int x, float sigma) {
    float coeff = 1.0f / (sqrt(2.0f * M_PI_F) * sigma);
    float exponent = -float(x * x) / (2.0f * sigma * sigma);
    return coeff * exp(exponent);
}

// ---- cuda_phys_cam_defocus_blur_kernel ----
// Depth-driven Gaussian blur. NB this is a GATHER using the *centre* pixel's blur diameter,
// exactly like the CUDA kernel (which reads buf_in around px_idx weighted by px_idx's kernel).
kernel void physCamDefocusBlur(device const float4* bufIn [[buffer(0)]],
                               device float4* bufOut [[buffer(1)]],
                               constant PhysCamParams& p [[buffer(2)]],
                               uint gid [[thread_position_in_grid]]) {
    uint pixelNum = p.width * p.height;
    if (gid >= pixelNum) return;

    int kernel_size = 0;
    float d = bufIn[gid].w;                       // distance, [m]
    float denom = p.N * p.C * d * (p.U - p.f);
    if (d > 1e-9f && fabs(denom) > 1e-20f) {
        kernel_size = int(ceil(p.f * p.f * fabs(d - p.U) / denom));
        kernel_size = abs(kernel_size);
    }

    float3 outc = float3(0.0f);
    if (kernel_size > 1) {
        kernel_size = max(1, int(p.defocusGain * float(kernel_size) + p.defocusBias));
        if ((kernel_size % 2) == 0) kernel_size += 1;   // ensure odd
        int r = (kernel_size - 1) / 2;
        int x_src = int(gid % p.width);
        int y_src = int(gid / p.width);
        int x_min = max(0, x_src - r);
        int x_max = min(int(p.width) - 1, x_src + r);
        int y_min = max(0, y_src - r);
        int y_max = min(int(p.height) - 1, y_src + r);
        float sigma = float(kernel_size) / 6.0f;
        for (int y = y_min; y <= y_max; ++y) {
            float gy = Gaussian1D(y - y_src, sigma);
            for (int x = x_min; x <= x_max; ++x) {
                float w = gy * Gaussian1D(x - x_src, sigma);
                outc += bufIn[uint(y) * p.width + uint(x)].xyz * w;
            }
        }
    } else {
        outc = bufIn[gid].xyz;
    }
    bufOut[gid] = float4(outc, 1.0f);   // A channel set to 1.0, as in the CUDA kernel
}

// ---- cuda_phys_cam_vignetting_kernel ----
// E <- E * (1 - G_vignet + G_vignet * (cos(theta))^4)
kernel void physCamVignetting(device float4* io [[buffer(0)]],
                              constant PhysCamParams& p [[buffer(1)]],
                              uint gid [[thread_position_in_grid]]) {
    if (gid >= p.width * p.height) return;
    float col = float(gid % p.width);
    float row = float(gid / p.width);
    // NB both x and y are normalized by img_w (not img_h) -- matches phys_cam_ops.cu exactly.
    float x = (col - float(p.width - 1) / 2.0f) / float(p.width) * p.L;
    float y = (row - float(p.height - 1) / 2.0f) / float(p.width) * p.L;
    float f = max(fabs(p.f), 1e-12f);
    float c = cos(atan(sqrt(x * x + y * y) / f));
    float gain = 1.0f - p.GVignet + p.GVignet * c * c * c * c;
    io[gid] = float4(io[gid].xyz * gain, io[gid].w);
}

// ---- cuda_phys_cam_aggregator_kernel ----
// [W/m^2] x [sec] x [m^2] = [J]:  E <- E * G * P / N^2 * C^2 * t * QE
kernel void physCamAggregator(device float4* io [[buffer(0)]],
                              constant PhysCamParams& p [[buffer(1)]],
                              uint gid [[thread_position_in_grid]]) {
    if (gid >= p.width * p.height) return;
    float n2 = max(1e-12f, p.N * p.N);
    float scale = p.GAggregator * p.P / n2 * p.C * p.C * p.t;
    io[gid] = float4(io[gid].xyz * scale * p.qe.xyz, io[gid].w);
}

// ---- cuda_phys_cam_noise_kernel ----
// I = L + N_read,  L ~ Poisson(mu + D*t) approximated (as in the CUDA kernel) by
// mu + D*t + Normal(0,1)*gain*sqrt(mu + D*t), then + Normal(0,sigma_read).
// curand's XORWOW stream cannot be reproduced here; two independent PCG streams
// (shot/dark and FPN/read) are used instead, so the DISTRIBUTIONS match but the
// individual samples do not.
static inline uint pcg(thread uint& s) {
    s = s * 747796405u + 2891336453u;
    uint w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
    return (w >> 22u) ^ w;
}
static inline float rndf(thread uint& s) { return float(pcg(s)) * (1.0f / 4294967296.0f); }
static inline float gaussf(thread uint& s) {
    float u1 = max(rndf(s), 1e-6f), u2 = rndf(s);
    return sqrt(-2.0f * log(u1)) * cos(6.28318530718f * u2);   // Box-Muller
}

kernel void physCamNoise(device float4* io [[buffer(0)]],
                         constant PhysCamParams& p [[buffer(1)]],
                         uint gid [[thread_position_in_grid]]) {
    if (gid >= p.width * p.height) return;
    uint sShot = p.shotSeed ^ (gid * 2654435761u) ^ (p.frame * 2246822519u);
    uint sFPN  = p.fpnSeed  ^ (gid * 2654435761u) ^ (p.frame * 3266489917u);
    pcg(sShot); pcg(sFPN);   // warm up

    float3 e = io[gid].xyz;
    float3 dark = p.darkCurrent.xyz, ng = p.noiseGain.xyz, sr = p.sigmaRead.xyz;
    for (int ch = 0; ch < 3; ++ch) {
        float v = e[ch] + dark[ch] * p.t;                 // mean number of electrons
        v += gaussf(sShot) * ng[ch] * sqrt(max(0.0f, v));  // shot + dark/hot-pixel noise
        v += gaussf(sFPN) * sr[ch];                        // FPN + readout noise
        e[ch] = v;
    }
    io[gid] = float4(e, io[gid].w);
}

// ---- cuda_phys_cam_expsr2dv_kernel_gamma (crf_type 0) ----
// I = a * (lg(E))^gamma + b
kernel void physCamExpsr2dvGamma(device const float4* bufIn [[buffer(0)]],
                                 device float4* bufOut [[buffer(1)]],
                                 constant PhysCamParams& p [[buffer(2)]],
                                 uint gid [[thread_position_in_grid]]) {
    if (gid >= p.width * p.height) return;
    float4 in = bufIn[gid];
    float3 g = p.gains.xyz, b = p.biases.xyz;
    float3 o;
    for (int ch = 0; ch < 3; ++ch) {
        float e = max(1e-12f, p.ISO * in[ch]);
        o[ch] = g[ch] * pow(log2(e), p.gamma) + b[ch];
    }
    bufOut[gid] = float4(o, in.w);
}

// ---- cuda_phys_cam_expsr2dv_kernel_sigmoid (crf_type 1) ----
// I = 1 / (1 + exp(-(a * ISO * E + b)))
kernel void physCamExpsr2dvSigmoid(device const float4* bufIn [[buffer(0)]],
                                   device float4* bufOut [[buffer(1)]],
                                   constant PhysCamParams& p [[buffer(2)]],
                                   uint gid [[thread_position_in_grid]]) {
    if (gid >= p.width * p.height) return;
    float4 in = bufIn[gid];
    float3 v = p.gains.xyz * p.ISO * in.xyz + p.biases.xyz;
    float3 o = 1.0f / (1.0f + exp(-v));
    bufOut[gid] = float4(o, 1.0f);   // the CUDA sigmoid kernel forces A = 1.0
}

// ---- cuda_phys_cam_expsr2dv_kernel_linear (crf_type 2) ----
// I = a * ISO * E + b
kernel void physCamExpsr2dvLinear(device const float4* bufIn [[buffer(0)]],
                                  device float4* bufOut [[buffer(1)]],
                                  constant PhysCamParams& p [[buffer(2)]],
                                  uint gid [[thread_position_in_grid]]) {
    if (gid >= p.width * p.height) return;
    float4 in = bufIn[gid];
    float3 o = p.gains.xyz * p.ISO * in.xyz + p.biases.xyz;
    bufOut[gid] = float4(o, in.w);
}
)MSLGEN";

#endif
