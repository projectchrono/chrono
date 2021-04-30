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
// Authors: Han Wang, Asher Elmquist
// =============================================================================
//
// RT kernels for tracing and measureing depth for a radar
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"

extern "C" __global__ void __raygen__radar() {
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const RadarParameters& radar = raygen->specific.radar;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float2 d = (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f -
               make_float2(1.f);  //[-1,1]
    float theta = d.x * radar.hFOV / 2.0;
    float phi = radar.min_vert_angle + (d.y * .5 + .5) * (radar.max_vert_angle - radar.min_vert_angle);
    float xy_proj = cos(phi);
    float z = sin(phi);
    float y = xy_proj * sin(theta);
    float x = xy_proj * cos(theta);

    const float t_frac = idx.x / (float)screen.x;
    const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent
    float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
    float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
    float3 forward;
    float3 left;
    float3 up;
    basis_from_quaternion(ray_quat, forward, left, up);
    float3 ray_direction = normalize(forward * x + left * y + up * z);
    // PerRayData_radar prd_radar = make_radar_data(0, 0.f);

    PerRayData_radar prd_radar = default_radar_prd();
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd_radar, opt1, opt2);
    unsigned int raytype = (unsigned int)RADAR_RAY_TYPE;
    optixTrace(params.root, ray_origin, ray_direction, radar.clip_near, 1.5f * radar.max_distance, t_traverse,
               OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0u, 1u, 0u, opt1, opt2,raytype);
    radar.frame_buffer[image_index] = make_float2(prd_radar.range, prd_radar.rcs);
}
