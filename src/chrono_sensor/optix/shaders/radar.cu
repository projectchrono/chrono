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
#include "chrono_sensor/optix/ChOptixDefinitions.h"

extern "C" __global__ void __raygen__radar() {
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const RadarParameters& radar = raygen->specific.radar;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float2 d = (make_float2(idx.x, idx.y) + make_float2(0.5, 0.5)) / make_float2(screen.x, screen.y) * 2.f -
               make_float2(1.f);  //[-1,1]
    float theta = d.x * radar.hFOV / 2.0;
    float phi = -radar.vFOV / 2 + (d.y * .5 + .5) * (radar.vFOV);
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

    PerRayData_radar prd_radar = default_radar_prd();
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd_radar, opt1, opt2);
    unsigned int raytype = (unsigned int)RADAR_RAY_TYPE;
    optixTrace(params.root, ray_origin, ray_direction, radar.clip_near, 1.5f * radar.max_distance, t_traverse,
               OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0u, 1u, 0u, opt1, opt2,raytype);
    
    float3 vel_global;
    // removing stationary object ray hits
    if (abs(prd_radar.velocity.x) > 0 || abs(prd_radar.velocity.y) > 0 || abs(prd_radar.velocity.z) > 0){
        vel_global = prd_radar.velocity - radar.velocity;

    } else{
        vel_global = make_float3(0,0,0);
    }

    float3 vel_radar_frame =  make_float3(Dot(forward, vel_global), Dot(left, vel_global), Dot(up, vel_global));

    int hIndex = image_index % screen.x;
    int vIndex = image_index / screen.x;

    float azimuth = (hIndex / (float)(screen.x)) * radar.hFOV - radar.hFOV / 2.;
    float elevation = (vIndex / (float)(screen.y)) * (radar.vFOV) - radar.vFOV / 2;
        
    radar.frame_buffer[8 * image_index] = prd_radar.range;
    radar.frame_buffer[8 * image_index + 1] = azimuth;
    radar.frame_buffer[8 * image_index + 2] = elevation; // x velocity
    radar.frame_buffer[8 * image_index + 3] = vel_radar_frame.x; // y velocity
    radar.frame_buffer[8 * image_index + 4] = vel_radar_frame.y; // z velocity
    radar.frame_buffer[8 * image_index + 5] = vel_radar_frame.z; // z velocity
    radar.frame_buffer[8 * image_index + 6] = prd_radar.rcs;
    radar.frame_buffer[8 * image_index + 7] = prd_radar.objectId; // objectId
//    printf("%f %f\n", prd_radar.range, azimuth);

}
