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
// Authors: Asher Elmquist, Han Wang
// =============================================================================
//
// RT kernels for box geometries
//
// =============================================================================

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"

static __device__ __inline__ float NormalDist(const float& NdH, const float& roughness) {
    float rough_sqr = roughness * roughness;
    float den_2 = NdH * NdH * (rough_sqr - 1.f) + 1.f;
    float denominator = den_2 * den_2;
    return rough_sqr / denominator;
}

// algorithm reference: https://www.gdcvault.com/play/1024478/PBR-Diffuse-Lighting-for-GGX
static __device__ __inline__ float HammonSmith(float NdV, float NdL, const float& roughness) {
    NdV = abs(NdV);
    NdL = abs(NdL);
    float denominator = lerp(2.f * NdV * NdL, NdL + NdV, roughness);
    return 0.5f / denominator;
}

extern "C" __global__ void __closesthit__camera_shader() {
    const MaterialRecordParameters* mat_params = (MaterialRecordParameters*)optixGetSbtDataPointer();

    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir = normalize(optixGetWorldRayDirection());  // this may be modified by the scaling transform
    const float ray_dist = optixGetRayTmax();

    float3 object_normal;
    float2 uv;
    float3 tangent;

    unsigned int material_id = mat_params->material_pool_id;

    // check if we hit a triangle
    if (optixIsTriangleHit()) {
        GetTriangleData(object_normal, material_id, uv, tangent, mat_params->mesh_pool_id);
    } else {
        object_normal = make_float3(int_as_float(optixGetAttribute_0()), int_as_float(optixGetAttribute_1()),
                                    int_as_float(optixGetAttribute_2()));
        uv = make_float2(int_as_float(optixGetAttribute_3()), int_as_float(optixGetAttribute_4()));
        tangent = make_float3(int_as_float(optixGetAttribute_5()), int_as_float(optixGetAttribute_6()),
                              int_as_float(optixGetAttribute_7()));
    }

    const MaterialParameters& mat = params.material_pool[material_id];

    if (mat.kn_tex) {
        float3 bitangent = normalize(Cross(object_normal, tangent));
        const float4 tex = tex2D<float4>(mat.kn_tex, uv.x, uv.y);
        float3 normal_delta = make_float3(tex.x, tex.y, tex.z) * 2.f - make_float3(1.f);
        object_normal =
            normalize(normal_delta.x * tangent + normal_delta.y * bitangent + normal_delta.z * object_normal);
    }

    float3 world_normal = normalize(optixTransformNormalFromObjectToWorldSpace(object_normal));

    if (Dot(world_normal, -ray_dir) < 0) {
        world_normal = -world_normal;
    }

    float3 hit_point = ray_orig + ray_dir * ray_dist;

    float3 subsurface_albedo = mat.Kd;
    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x, uv.y);
        subsurface_albedo = make_float3(tex.x, tex.y, tex.z);
    }

    PerRayData_camera* prd_camera = getCameraPRD();

    //=================
    // Refracted color
    //=================
    float3 refracted_color = make_float3(0);
    if (mat.transparency < 0.99f) {
        float refract_importance = prd_camera->contrib_to_first_hit * (1 - mat.transparency);
        if (refract_importance > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_refraction = default_camera_prd();
            prd_refraction.contrib_to_first_hit = refract_importance;
            prd_refraction.rng = prd_camera->rng;
            prd_refraction.depth = prd_camera->depth + 1;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_refraction, opt1, opt2);

            // make_camera_data(make_float3(0), refract_importance, prd_camera.rnd, prd_camera.depth + 1);
            float3 refract_dir = refract(optixGetWorldRayDirection(), world_normal, 1.f, 1.f);

            optixTrace(params.root, hit_point, refract_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, CAMERA_RAY_TYPE, RAY_TYPE_COUNT, CAMERA_RAY_TYPE,
                       opt1, opt2);
            refracted_color = prd_refraction.color;
        }
    }

    // query roughness and metalic values
    float roughness = mat.roughness;
    if (mat.roughness_tex) {
        roughness = tex2D<float>(mat.roughness_tex, uv.x, uv.y);
    }
    float metallic = mat.metallic;
    if (mat.metallic_tex) {
        metallic = tex2D<float>(mat.metallic_tex, uv.x, uv.y);
    }

    //=================
    // Diffuse color
    //=================
    float3 diffuse_color = make_float3(0.0f);
    // iterate through the lights
    for (int i = 0; i < params.num_lights; i++) {
        PointLight l = params.lights[i];
        float dist_to_light = Length(l.pos - hit_point);
        if (dist_to_light < 2 * l.max_range) {
            float3 dir_to_light = normalize(l.pos - hit_point);
            float NdL = Dot(world_normal, dir_to_light);

            // if we think we can see the light, let's see if we are correct
            if (NdL > 0.0f) {
                // check shadows
                PerRayData_shadow prd_shadow = default_shadow_prd();
                prd_shadow.depth = prd_camera->depth + 1;
                prd_shadow.ramaining_dist = dist_to_light;
                unsigned int opt1;
                unsigned int opt2;
                pointer_as_ints(&prd_shadow, opt1, opt2);
                optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light, optixGetRayTime(),
                           OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, SHADOW_RAY_TYPE, RAY_TYPE_COUNT,
                           SHADOW_RAY_TYPE, opt1, opt2);

                float3 light_attenuation = prd_shadow.attenuation;

                float point_light_falloff =
                    (l.max_range * l.max_range / (dist_to_light * dist_to_light + l.max_range * l.max_range));

                float3 incoming_light_ray = l.color * light_attenuation * point_light_falloff * NdL;

                if (fmaxf(incoming_light_ray) > 0.0f) {
                    float3 halfway = normalize(dir_to_light - ray_dir);
                    float NdV = Dot(world_normal, -ray_dir);
                    float NdH = Dot(world_normal, halfway);
                    float VdH = Dot(-ray_dir, halfway);

                    float3 F = make_float3(0.5f);
                    // === dielectric workflow
                    if (metallic > 0) {
                        float3 default_dielectrics_F0 = make_float3(0.04f);
                        F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                        subsurface_albedo =
                            (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection
                    } else {                                     // default to specular workflow
                        float3 F0 = mat.Ks * 0.08f;
                        F = fresnel_schlick(VdH, 5.f, F0,
                                            make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
                    }

                    // === Fresnel_at_0 to Fresnel_at_90 workflow
                    // F = fresnel_schlick(VdH, mat.fresnel_exp, make_float3(mat.fresnel_min),
                    //                     make_float3(mat.fresnel_max));

                    // === IoF workflow
                    // float3 ratio = (iof1 - iof2) / (iof1 + iof2); // one of them should be air (iof = 1)
                    // float3 F0 = ratio * ratio;
                    // F = fresnel_schlick(NdV, 5, F0, make_float3(1));

                    diffuse_color += (make_float3(1.f) - F) * subsurface_albedo * incoming_light_ray;
                    float D = NormalDist(NdH, roughness);        // 1/pi omitted
                    float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
                    float3 f_ct = F * D * G;
                    diffuse_color += f_ct * incoming_light_ray;
                }
            }
        }
    }

    // In ambient light mode

    float NdV = Dot(world_normal, -ray_dir);
    diffuse_color = diffuse_color + params.ambient_light_color * make_float3(NdV) * subsurface_albedo;

    if (prd_camera->depth == 2) {
        prd_camera->albedo = subsurface_albedo;
        prd_camera->normal = world_normal;
    }

    // refracted color contribution to this hit is accounted for when tracing refraction
    prd_camera->color = diffuse_color * (mat.transparency) * prd_camera->contrib_to_first_hit + refracted_color;

    float3 next_dir;
    if (prd_camera->use_gi) {
        // sample hemisphere for next ray when using global illumination
        float z1 = curand_uniform(&prd_camera->rng) * 2.f - 1.f;  //-1,1
        float z2 = curand_uniform(&prd_camera->rng) * 2.f - 1.f;  //-1,1
        next_dir = sample_hemisphere_dir(z1, z2, world_normal);
    } else {
        next_dir = reflect(ray_dir, world_normal);
    }
    float NdL = Dot(world_normal, next_dir);
    float3 halfway = normalize(next_dir - ray_dir);

    float NdH = Dot(world_normal, halfway);
    float VdH = Dot(-ray_dir, halfway);  // Same as LdH
    // NdV = Dot(world_normal, -ray_dir);

    // === Metallic workflow
    // float3 default_dielectrics_F0 = make_float3(0.04f);
    // float3 F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
    // subsurface_albedo = (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection

    // === Ks Workflow
    float3 F = fresnel_schlick(VdH, 5.f, mat.Ks * 0.08f, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
    // === Fresnel_at_0 to Fresnel_at_90 workflow
    // float3 F = fresnel_schlick(VdH, fresnel_exp, make_float3(fresnel_min), make_float3(fresnel_max));

    float3 next_contrib_to_first_hit = (make_float3(1.f) - F) * subsurface_albedo * NdL;

    // float D = NormalDist(NdH, roughness);  // 1/pi omitted
    float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
    float D = NormalDist(NdH, roughness) / CUDART_PI_F;
    // float G = HammonSmith(NdV, NdL, roughness) / (4 * NdV * NdL);
    float3 f_ct = F * D * G;
    next_contrib_to_first_hit += f_ct * NdL;

    if (!prd_camera->use_gi) {  // we need to account for the fact we didn't randomly sample the direction (heuristic
                                // since this is not physical)
        next_contrib_to_first_hit = (1.f - roughness) * next_contrib_to_first_hit / (2 * CUDART_PI_F);
    }

    // contribution should never exceed 1 or esle we are creating energy on reflection
    next_contrib_to_first_hit =
        clamp(next_contrib_to_first_hit, make_float3(0.f), make_float3(1.f)) * prd_camera->contrib_to_first_hit;

    // russian roulette termination of rays
    // if (prd_camera->depth >= 3 && curand_uniform(&prd_camera->rng) >= luminance(next_contrib_to_first_hit)) {
    //     return;
    // }

    if (luminance(next_contrib_to_first_hit) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
        PerRayData_camera prd_reflection = default_camera_prd();
        prd_reflection.contrib_to_first_hit = luminance(next_contrib_to_first_hit);
        prd_reflection.rng = prd_camera->rng;
        prd_reflection.depth = prd_camera->depth + 1;
        prd_reflection.use_gi = prd_camera->use_gi;
        unsigned int opt1, opt2;
        pointer_as_ints(&prd_reflection, opt1, opt2);

        optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, CAMERA_RAY_TYPE, RAY_TYPE_COUNT, CAMERA_RAY_TYPE, opt1,
                   opt2);
        prd_camera->color += prd_reflection.color;  // accumulate indirect lighting color
    }
}