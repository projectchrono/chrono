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
// Authors: Bo-Hsun Chen (modified from Asher Elmquist's camera.cu)
// =============================================================================
//
// Normal camera ray launch kernels
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"

/// Default of normal camera per ray data (PRD)
__device__ __inline__ PerRayData_normalCamera DefaultNormalCameraPRD() {
    PerRayData_normalCamera prd = {};
    prd.normal = make_float3(0.f, 0.f, 0.f);
    return prd;
};

/// Ray generation program for normal camera
extern "C" __global__ void __raygen__normal_camera() {
    const RaygenParameters* raygen = (RaygenParameters*) optixGetSbtDataPointer();
    const NormalCameraParameters& camera = raygen->specific.normalCamera;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float2 d =
        (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f - make_float2(1.f);
    d.y *= (float)(screen.y) / (float)(screen.x);  // correct for the aspect ratio

    // FOV lens model
    if (camera.lens_model == FOV_LENS && ((d.x) > 1e-5 || abs(d.y) > 1e-5)) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float2 d_normalized = d / focal;
        float rd = sqrtf(d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y);
        float ru = tanf(rd * camera.hFOV) / (2 * tanf(camera.hFOV / 2.0));
        d = d_normalized * (ru / rd) * focal;

    } // radial lens model
    else if (camera.lens_model == RADIAL) {
        float focal = 1.f / tanf(camera.hFOV / 2.0);
        float recip_focal = tanf(camera.hFOV / 2.0);
        float2 d_normalized = d * recip_focal;
        float rd2 = d_normalized.x * d_normalized.x + d_normalized.y * d_normalized.y;
        float distortion_ratio = radial_function(rd2,camera.lens_parameters);
        d = d_normalized * distortion_ratio * focal;
    }

    float t_frac = 0.f;
    if (camera.rng_buffer) {
        t_frac = curand_uniform(&camera.rng_buffer[image_index]);  // 0-1 between start and end time of the camera (chosen here)
    }
    const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent

    float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
    float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
    // float3 ray_origin = raygen->pos0;
    // float4 ray_quat = raygen->rot0;
    const float h_factor = camera.hFOV / CUDART_PI_F * 2.0;
    float3 forward;
    float3 left;
    float3 up;

    basis_from_quaternion(ray_quat, forward, left, up);
    float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

    PerRayData_normalCamera prd = DefaultNormalCameraPRD();
    
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd, opt1, opt2);
    unsigned int raytype = static_cast<unsigned int>(RayType::NORMAL_RAY_TYPE);
    optixTrace(params.root, ray_origin, ray_direction, params.scene_epsilon, 1e16f, t_traverse, OptixVisibilityMask(1),
               OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

    camera.frame_buffer[image_index] = make_float3(prd.normal.x, prd.normal.y, prd.normal.z);
}
