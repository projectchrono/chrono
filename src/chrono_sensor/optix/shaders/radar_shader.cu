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
// Authors: Asher Elmquist, Han Wang, Yan Xiao
// =============================================================================
//
// RADAR shader
//
// =============================================================================

#ifndef RADAR_SHADER_CU
#define RADAR_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.h"

__device__ __inline__ PerRayData_radar* GetRadarPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_radar*>(ints_as_pointer(opt0, opt1));
}


static __device__ __inline__ void RadarShader(PerRayData_radar* prd_radar,
                                              const MaterialParameters& mat,
                                              const float3& world_normal,
                                              const float2& uv,
                                              const float3& tangent,
                                              const float& ray_dist,
                                              const float3& ray_orig,
                                              const float3& ray_dir,
                                              const float3& translational_velocity,
                                              const float3& angular_velocity,
                                              const float& objectId) {
    prd_radar->range = ray_dist;
    prd_radar->rcs = mat.radar_backscatter * abs(Dot(world_normal, -ray_dir));
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float3 origin = optixTransformPointFromObjectToWorldSpace(make_float3(0, 0, 0));
    float3 r = hit_point - origin;

    prd_radar->velocity = translational_velocity + Cross(angular_velocity, r);
    prd_radar->objectId = objectId;
}

# endif  // RADAR_SHADER_CU
