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
// RT kernels for shadows
//
// =============================================================================

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"

extern "C" __global__ void __closesthit__shadow_shader() {
    const MaterialRecordParameters* mat_params = (MaterialRecordParameters*)optixGetSbtDataPointer();

    float3 ray_orig = optixGetWorldRayOrigin();
    float3 ray_dir = optixGetWorldRayDirection();
    float ray_dist = optixGetRayTmax();

    float3 object_normal;
    float2 uv;
    float3 tangent;

    unsigned int material_id = mat_params->material_pool_id;

    // check if we hit a triangle to see if there is a material offset
    if (optixIsTriangleHit()) {
        GetTriangleData(object_normal, material_id, uv, tangent, mat_params->mesh_pool_id);
    }

    const MaterialParameters& mat = params.material_pool[material_id];

    PerRayData_shadow* prd = getShadowPRD();

    float atten = 1.f - mat.transparency;  // TODO: figure out the attenuation from the material transparency

    // if the occlusion amount is below the
    prd->attenuation = prd->attenuation * atten;

    if (fmaxf(prd->attenuation) > params.importance_cutoff && prd->depth + 1 < params.max_depth) {
        PerRayData_shadow prd_shadow = default_shadow_prd();
        prd_shadow.attenuation = prd->attenuation;
        prd_shadow.depth = prd->depth + 1;
        prd_shadow.ramaining_dist = prd->ramaining_dist - ray_dist;
        unsigned int opt1, opt2;
        pointer_as_ints(&prd_shadow, opt1, opt2);

        float3 hit_point = ray_orig + ray_dist * ray_dir;
        optixTrace(params.root, hit_point, ray_dir, params.scene_epsilon, prd_shadow.ramaining_dist, optixGetRayTime(),
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, SHADOW_RAY_TYPE, RAY_TYPE_COUNT, SHADOW_RAY_TYPE, opt1,
                   opt2);

        prd->attenuation = prd_shadow.attenuation;
    }

    // prd->attenuation = make_float3(0.f, 0.f, 0.f);

    // setShadowPRD(prd);
}

// extern "C" __global__ void __anyhit__shadow_shader() {
//     PerRayData_shadow prd = getShadowPRD();
//     prd.attenuation = make_float3(0.f, 0.f, 0.f);
//     // setShadowPRD(prd);
// }