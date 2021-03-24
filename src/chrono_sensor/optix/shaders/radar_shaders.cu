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
// RT kernels for radar shading
//
// =============================================================================
#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"

extern "C" __global__ void __closesthit__radar_shader() {
    float3 ray_orig = optixGetWorldRayOrigin();
    float3 ray_dir = optixGetWorldRayDirection();
    float ray_dist = optixGetRayTmax();
    const MaterialRecordParameters* mat_params = (MaterialRecordParameters*)optixGetSbtDataPointer();

    float3 forward_normal;
    float2 uv;
    float3 tangent;

    unsigned int material_id = mat_params->material_pool_id;

    if (optixIsTriangleHit()) {
        GetTriangleData(forward_normal, material_id, uv, tangent, mat_params->mesh_pool_id);
    } else {
        forward_normal = make_float3(int_as_float(optixGetAttribute_0()), int_as_float(optixGetAttribute_1()),
                                     int_as_float(optixGetAttribute_2()));
        uv = make_float2(int_as_float(optixGetAttribute_3()), int_as_float(optixGetAttribute_4()));
        tangent = make_float3(int_as_float(optixGetAttribute_5()), int_as_float(optixGetAttribute_6()),
                              int_as_float(optixGetAttribute_7()));
    }

    const MaterialParameters& mat = params.material_pool[material_id];

    if (mat.kn_tex) {
        float3 bitangent = normalize(Cross(forward_normal, tangent));
        const float4 tex = tex2D<float4>(mat.kn_tex, uv.x, uv.y);
        float3 normal_delta = make_float3(tex.x, tex.y, tex.z) * 2.f - make_float3(1.f);
        forward_normal =
            normalize(normal_delta.x * tangent + normal_delta.y * bitangent + normal_delta.z * forward_normal);
    }

    float3 world_normal = normalize(optixTransformNormalFromObjectToWorldSpace(forward_normal));

    if (Dot(world_normal, -ray_dir) < 0) {
        world_normal = -world_normal;
    }

    PerRayData_radar* prd_radar = getRadarPRD();
    prd_radar->range = ray_dist;
    prd_radar->rcs = mat.radar_backscatter * abs(Dot(forward_normal, -ray_dir));
}
