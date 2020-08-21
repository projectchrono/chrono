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
 #define NOMINMAX
#endif

#include <optix.h>
#include <optixu/optixu_math_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
#include "chrono_sensor/scene/lights.h"

using namespace optix;

rtDeclareVariable(PerRayData_camera, prd_camera, rtPayload, );
rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float3, tangent_vector, attribute tangent_vector, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );

rtDeclareVariable(float3, ambient_light_color, , );
rtDeclareVariable(float3, Ka, , );
rtDeclareVariable(float3, Kd, , );
rtDeclareVariable(float3, Ks, , );
rtDeclareVariable(float, transparency, , );
rtDeclareVariable(float, phong_exp, , );
rtDeclareVariable(float, roughness, , );
rtDeclareVariable(float, fresnel_exp, , );
rtDeclareVariable(float, fresnel_min, , );
rtDeclareVariable(float, fresnel_max, , );

rtDeclareVariable(float2, texcoord, attribute texcoord, );
rtDeclareVariable(int, has_texture, , );
rtDeclareVariable(int, has_normal_map, , );
rtTextureSampler<float4, 2> Kd_map;
rtTextureSampler<float4, 2> normal_map;

rtDeclareVariable(rtObject, root_node, , );
rtDeclareVariable(float, scene_epsilon, , );
rtDeclareVariable(float, max_scene_distance, , );
rtDeclareVariable(float, importance_cutoff, , );
rtDeclareVariable(int, max_depth, , );

rtBuffer<PointLight> lights;

static __device__ float NormalDist(float NdH, float roughness) {
    float numerator = roughness * roughness;
    float den_2 = NdH * NdH * (numerator - 1) + 1;
    float denominator = den_2 * den_2;

    return numerator / denominator;
}

static __device__ float GeomSmithSchlick(float NdV, float NdL, float roughness) {
    float r_remap = (roughness + 1) * (roughness + 1) / 8;

    float s_ndv = NdV / (NdV * (1 - r_remap) + r_remap);
    float s_ndl = NdL / (NdL * (1 - r_remap) + r_remap);

    return s_ndv * s_ndl;
}

// simplest camera shader that colors by object normal
RT_PROGRAM void normal_shader() {
    // set the result equal to distance to the intersection
    prd_camera.color = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, shading_normal)) * 0.5f + 0.5f;
}

// camera shader that colors by kd_map
RT_PROGRAM void diffuse_shader() {
    prd_camera.color = make_float3(tex2D(Kd_map, texcoord.x, texcoord.y));
}

// PBR Shader based on Cook-Torrence model including specular and diffuse coloring, reflective and refractive
// componenets when necessary
RT_PROGRAM void pbr_shader() {
    float3 forward_normal = shading_normal;

    // if has normal_map use the normal map for normal
    if (has_normal_map) {
        // Transform TBN to world space
        float3 tangent = normalize(tangent_vector);
        float3 bitangent = normalize(cross(forward_normal, tangent));

        // Extract normal_delta
        float3 normal_delta = make_float3(tex2D(normal_map, texcoord.x, texcoord.y)) * 2 - make_float3(1);

        // Calculate final normal TBN * normal delta
        forward_normal = -(tangent * normal_delta.x + bitangent * normal_delta.y + forward_normal * normal_delta.z);
    }

    forward_normal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, forward_normal));

    if (dot(forward_normal, -ray.direction) < 0) {
        forward_normal = -forward_normal;
    }

    float3 hit_point = ray.origin + ray.direction * t_hit;  // + 0.01 * forward_normal;

    // get Kd either from color or from texture
    float3 tmp_kd = Kd;
    if (has_texture) {
        tmp_kd = make_float3(tex2D(Kd_map, texcoord.x, texcoord.y));
    }

    // ==========================================================
    // Ambient Color - fudge factor to prevent super dark shadows
    // ==========================================================
    prd_camera.color = make_float3(0);  // = tmp_kd * ambient_light_color;

    //=================
    // Reflected color
    //=================
    float3 reflect_amount = make_float3(0);
    if (roughness < .99) {
        reflect_amount = Ks * fresnel_schlick(dot(forward_normal, -ray.direction), fresnel_exp,
                                              make_float3(fresnel_min), make_float3(fresnel_max));

        float reflect_importance = prd_camera.importance * luminance(reflect_amount);

        if (reflect_importance > importance_cutoff && prd_camera.depth < max_depth) {
            PerRayData_camera prd_reflection =
                make_camera_data(make_float3(0), reflect_importance, prd_camera.depth + 1);

            float3 reflect_dir = reflect(ray.direction, forward_normal);
            Ray reflection_ray(hit_point, reflect_dir, CAMERA_RAY_TYPE, scene_epsilon, max_scene_distance);
            rtTrace(root_node, reflection_ray, prd_reflection, RT_RAY_FLAG_DISABLE_ANYHIT);
            prd_camera.color += reflect_importance * prd_reflection.color;
        }
    }

    //=================
    // Refracted color
    //=================
    float refract_importance = 0;
    if (transparency < .99) {
        refract_importance = prd_camera.importance * (1 - transparency) * (1 - luminance(reflect_amount));
        if (refract_importance > importance_cutoff && prd_camera.depth < max_depth) {
            PerRayData_camera prd_refraction =
                make_camera_data(make_float3(0), refract_importance, prd_camera.depth + 1);

            float3 refract_dir;
            refract(refract_dir, ray.direction, forward_normal, 1.f);
            Ray refraction_ray(hit_point, refract_dir, CAMERA_RAY_TYPE, scene_epsilon, max_scene_distance);
            rtTrace(root_node, refraction_ray, prd_refraction);
            prd_camera.color += refract_importance * prd_refraction.color;
        }
    }

    //=================
    // Diffuse color
    //=================

    // iterate through the lights
    for (int i = 0; i < lights.size(); i++) {
        PointLight l = lights[i];
        float dist_to_light = length(l.pos - hit_point);
        if (dist_to_light < 2 * l.max_range) {
            float3 dir_to_light = normalize(l.pos - hit_point);

            float NdL = dot(forward_normal, dir_to_light);

            // 0 if we already know there is a shadow, 1 if we might be able to see the light
            float3 light_attenuation = make_float3(static_cast<float>(NdL > 0.f));

            // if we think we can see the light, let's see if we are correct
            if (NdL > 0.0f) {
                light_attenuation = make_float3(NdL);

                // check shadows
                PerRayData_shadow prd_shadow;
                prd_shadow.attenuation = make_float3(1.0f);
                Ray shadow_ray(hit_point, dir_to_light, SHADOW_RAY_TYPE, scene_epsilon, dist_to_light);
                // rtTrace(root_node, shadow_ray, prd_shadow, RT_RAY_FLAG_TERMINATE_ON_FIRST_HIT);
                rtTrace(root_node, shadow_ray, prd_shadow, RT_RAY_FLAG_TERMINATE_ON_FIRST_HIT);

                light_attenuation = prd_shadow.attenuation;
            }

            // if any of our channels can see the light, let's calculate the contribution
            if (fmaxf(light_attenuation) > 0.0f) {
                float NdL_result = (.01f * l.max_range * l.max_range /
                                    (dist_to_light * dist_to_light + .01f * l.max_range * l.max_range)) *
                                   NdL;

                if (NdL_result > 1e-6f) {
                    // ==========================================
                    // Cook-Torrence Model for blending the light
                    // ==========================================

                    float3 halfway = normalize(dir_to_light - ray.direction);
                    float NdH = dot(forward_normal, halfway);
                    float NdV = dot(forward_normal, -ray.direction);

                    float N = NormalDist(NdH, roughness);
                    float D = GeomSmithSchlick(NdV, NdL, roughness);
                    float3 F = fresnel_schlick(dot(halfway, dir_to_light), fresnel_exp, make_float3(fresnel_min),
                                               make_float3(fresnel_max));
                    float3 f_ct = N * D * F / (4 * NdV * NdL);

                    // lighting equation for cook-torrence
                    float3 obj_color =
                        ((make_float3(1) - Ks) * tmp_kd + Ks * f_ct) * l.color * light_attenuation * NdL_result;
                    prd_camera.color += (1 - reflect_amount) * (1 - refract_importance) * obj_color;
                }
            }
        }
    }
    prd_camera.color = fmaxf(prd_camera.color, tmp_kd * ambient_light_color);
}
