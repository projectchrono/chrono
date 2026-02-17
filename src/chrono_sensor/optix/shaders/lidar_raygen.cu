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
// Authors: Asher Elmquist
// =============================================================================
//
// RT kernels for tracing and measureing depth for a scanning TOF lidar
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"

/// Default of LiDAR per ray data (PRD)
__device__ __inline__ PerRayData_lidar DefaultLidarPRD() {
    PerRayData_lidar prd = {
        0.f,  // default range
        0.f   // default intensity
    };
    return prd;
};

extern "C" __global__ void __raygen__lidar_single() {
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const LidarParameters& lidar = raygen->specific.lidar;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    float phi = (idx.y / (float)(max(1,screen.y-1))) * (lidar.max_vert_angle -  lidar.min_vert_angle) +  lidar.min_vert_angle;
    float theta = (idx.x / (float)(max(1,screen.x-1))) * lidar.hFOV - lidar.hFOV / 2.;

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
    
    PerRayData_lidar prd_lidar = DefaultLidarPRD();
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd_lidar, opt1, opt2);
    unsigned int raytype = (unsigned int)RayType::LIDAR_RAY_TYPE;
    optixTrace(params.root, ray_origin, ray_direction, lidar.clip_near, 1.5 * lidar.max_distance, t_traverse,
               OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

    lidar.frame_buffer[image_index] = make_float2(prd_lidar.range, prd_lidar.intensity);
}

extern "C" __global__ void __raygen__lidar_multi() {
    const RaygenParameters* raygen = (RaygenParameters*)optixGetSbtDataPointer();
    const LidarParameters& lidar = raygen->specific.lidar;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 screen = optixGetLaunchDimensions();
    const unsigned int image_index = screen.x * idx.y + idx.x;

    // global_beam_dims = number of horizontal and vertical samples
    const int2 global_beam_dims =
        make_int2(screen.x / (lidar.sample_radius * 2 - 1), screen.y / (lidar.sample_radius * 2 - 1));
    const int2 local_beam_dims = make_int2(lidar.sample_radius * 2 - 1, lidar.sample_radius * 2 - 1);

    // index of center of beam
    const int beam_index_x = idx.x / (lidar.sample_radius * 2 - 1);
    const int beam_index_y = idx.y / (lidar.sample_radius * 2 - 1);

    float beam_phi = (beam_index_y / (float)(max(1,global_beam_dims.y-1))) * (lidar.max_vert_angle -  lidar.min_vert_angle) +  lidar.min_vert_angle;
    float beam_theta = (beam_index_x / (float)(max(1,global_beam_dims.x-1))) * lidar.hFOV - lidar.hFOV / 2.;

    // index of local ray in beam,  0~sample_radius * 2 - 1, and sample_radius - 1 is index of center ray
    const int local_ray_index_x = idx.x % (lidar.sample_radius * 2 - 1);
    const int local_ray_index_y = idx.y % (lidar.sample_radius * 2 - 1);

    const float2 local_ray_id_fraction = (make_float2(local_ray_index_x, local_ray_index_y) + make_float2(0.5, 0.5)) /
                                             make_float2(local_beam_dims) * 2.f -
                                         make_float2(1.f);  //[-1,1]

    // relative theta and phi for local ray in beam
    float local_ray_theta;
    float local_ray_phi;

    // beam shape is rectangular
    if (lidar.beam_shape == LidarBeamShape::ELLIPTICAL) {
        local_ray_theta = local_ray_id_fraction.x * lidar.horiz_div_angle / 2.0;
        local_ray_phi = local_ray_id_fraction.y * lidar.vert_div_angle / 2.0;
        // beam shape is elliptical
    } else {  // defaulting to rectangular
        float angle = atan2(local_ray_id_fraction.y, local_ray_id_fraction.x);
        float ring = max(abs(local_ray_id_fraction.x), abs(local_ray_id_fraction.y));
        float2 axis = make_float2(lidar.vert_div_angle / 2.0 * ring, lidar.horiz_div_angle / 2.0 * ring);
        float radius;
        if (axis.y == 0 && axis.x == 0) {
            radius = 0;
        } else {
            radius = (axis.x * axis.y) /
                     sqrtf(axis.x * axis.x * sinf(angle) * sinf(angle) + axis.y * axis.y * cosf(angle) * cosf(angle));
        }
        local_ray_theta = radius * sinf(angle);
        local_ray_phi = radius * cosf(angle);
    }

    // carry on ray-tracing per ray
    const float theta = beam_theta + local_ray_theta;
    const float phi = beam_phi + local_ray_phi;

    const float xy_proj = cosf(phi);
    const float z = sinf(phi);
    const float y = xy_proj * sinf(theta);
    const float x = xy_proj * cosf(theta);

    const float t_frac = (beam_index_x / (float)global_beam_dims.x);
    const float t_traverse = raygen->t0 + t_frac * (raygen->t1 - raygen->t0);  // simulation time when ray is sent
    const float3 ray_origin = lerp(raygen->pos0, raygen->pos1, t_frac);
    const float4 ray_quat = nlerp(raygen->rot0, raygen->rot1, t_frac);
    float3 forward;
    float3 left;
    float3 up;
    basis_from_quaternion(ray_quat, forward, left, up);
    float3 ray_direction = normalize(forward * x + left * y + up * z);

    PerRayData_lidar prd_lidar = DefaultLidarPRD();  // make_lidar_data(0, 0.f);
    unsigned int opt1;
    unsigned int opt2;
    pointer_as_ints(&prd_lidar, opt1, opt2);
    unsigned int raytype = (unsigned int)RayType::LIDAR_RAY_TYPE;
    optixTrace(params.root, ray_origin, ray_direction, lidar.clip_near, 1.5 * lidar.max_distance, t_traverse,
               OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

    lidar.frame_buffer[image_index] = make_float2(prd_lidar.range, prd_lidar.intensity);
}