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

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>
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
rtDeclareVariable(float, roughness, , );
rtDeclareVariable(float, metallic, , );
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
    float rough_sqr = roughness * roughness;
    float den_2 = NdH * NdH * (rough_sqr - 1) + 1;
    float denominator = den_2 * den_2;

    return rough_sqr / denominator;
}

// https://www.gdcvault.com/play/1024478/PBR-Diffuse-Lighting-for-GGX
static __device__ float HammonSmith(float NdV, float NdL, float roughness) {
    NdV = abs(NdV);
    NdL = abs(NdL);
    float denominator = lerp(2 * NdV * NdL, NdL + NdV, roughness);
    return 0.5 / denominator;
}

static __device__ float checkerboard3(float3 loc) {
    loc += make_float3(0.001f);  // small epsilon so planes don't coincide with scene geometry
    float checkerboard_width = 40.f;
    int3 c;

    c.x = abs((int)floor((loc.x / checkerboard_width)));
    c.y = abs((int)floor((loc.y / checkerboard_width)));
    c.z = abs((int)floor((loc.z / checkerboard_width)));

    if ((c.x % 2) ^ (c.y % 2) ^ (c.z % 2))
        return 1.0f;
    return 0.0f;
}

// temporary using Optix Example !!!!
static __host__ __device__ __inline__ unsigned int lcg(unsigned int& prev) {
    const unsigned int LCG_A = 1664525u;
    const unsigned int LCG_C = 1013904223u;
    prev = (LCG_A * prev + LCG_C);
    return prev & 0x00FFFFFF;
}
// Generate random float in [0, 1)
static __host__ __device__ __inline__ float rnd(unsigned int& prev) {
    return ((float)lcg(prev) / (float)0x01000000);
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

    // Filp the normal when the ray hit the triangle on the back 
    // if (dot(forward_normal, -ray.direction) < 0) {
    //     forward_normal = -forward_normal;
    // }

    // Filp the normal when the ray hit the triangle on the back 
    forward_normal = faceforward(forward_normal, -ray.direction, forward_normal);

    float3 hit_point = ray.origin + ray.direction * t_hit;  // + 0.01 * forward_normal;

    // get subsurface_albedo either from color or from texture
    float3 subsurface_albedo = Kd;

    if (has_texture) {
        subsurface_albedo = make_float3(tex2D(Kd_map, texcoord.x, texcoord.y));
    }

    prd_camera.color = make_float3(0);
    //=================
    // punctual lights
    //=================
    float3 reflected_color = make_float3(0.0f);
    // iterate through the lights
    for (int i = 0; i < lights.size(); i++) {
        PointLight l = lights[i];
        float dist_to_light = length(l.pos - hit_point);

        if (dist_to_light >= 2 * l.max_range) {
            continue;
        }

        float3 dir_to_light = normalize(l.pos - hit_point);
        float NdL = dot(forward_normal, dir_to_light);

        // 0 if we already know there is a shadow, 1 if we might be able to see the light
        float3 light_attenuation = make_float3(static_cast<float>(NdL > 0.f));

        // if we think we can see the light, let's see if we are correct
        if (NdL > 0.0f) {
            // check shadows
            PerRayData_shadow prd_shadow;
            prd_shadow.attenuation = make_float3(1.0f);
            Ray shadow_ray(hit_point, dir_to_light, SHADOW_RAY_TYPE, scene_epsilon, dist_to_light);
            // rtTrace(root_node, shadow_ray, prd_shadow, RT_RAY_FLAG_TERMINATE_ON_FIRST_HIT);
            rtTrace(root_node, shadow_ray, prd_shadow, RT_RAY_FLAG_TERMINATE_ON_FIRST_HIT);
            light_attenuation = prd_shadow.attenuation;
        }

        float point_light_falloff = (l.max_range / (dist_to_light * dist_to_light + l.max_range));
        
        float3 incoming_light_ray = l.color * light_attenuation * point_light_falloff * NdL;
        
        // if any of our channels can see the light, let's calculate the contribution
        if (fmaxf(incoming_light_ray) > 0.0f) {
            
            float3 halfway = normalize(dir_to_light - ray.direction);
            float NdV = dot(forward_normal, -ray.direction);

            float NdH = dot(forward_normal, halfway);
            float VdH = dot(-ray.direction, halfway); // Same as LdH

            float3 F = make_float3(0.5f);
            // Metallic workflow
            if (false) {
                float3 default_dielectrics_F0 = make_float3(0.04f);
                
                F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                subsurface_albedo =
                    (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection
            }
            // Ks Workflow
            else if (true) {
                float3 F0 = Ks * 0.08f;
                F = fresnel_schlick(VdH, 5, F0, make_float3(1) /*make_float3(fresnel_max) it is usually 1*/);
            }
            // Fresnel_at_0 to Fresnel_at_90 workflow
            else if (false) {
                F = fresnel_schlick(VdH, fresnel_exp, make_float3(fresnel_min), make_float3(fresnel_max));
            }
            // IoF workflow
            else {
                // float3 ratio = (iof1 - iof2) / (iof1 + iof2); // one of them should be air (iof = 1)
                // float3 F0 = ratio * ratio;
                // F = fresnel_schlick(NdV, 5, F0, make_float3(1));
            }
            // ==========================================
            // diffuse reflection
            // ==========================================
            reflected_color += (1 - F) * subsurface_albedo * incoming_light_ray;
                   
            // ==========================================
            // specular reflection
            // ==========================================
            // Get parameters for specular reflection

            float D = NormalDist(NdH, roughness); // 1/pi omitted
            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
                
            float3 f_ct = F * D * G;

            reflected_color += f_ct * incoming_light_ray;
            
        }
    }
    prd_camera.color = reflected_color;
    float NdV = dot(forward_normal, -ray.direction);
    // In ambient light mode
    if (prd_camera.mode == FIRST_HIT) {
        prd_camera.color = prd_camera.color + ambient_light_color * Ka * make_float3(NdV) * subsurface_albedo ;
        // prd_camera.color = make_float3(roughness);
        return;
    }
    prd_camera.color *= prd_camera.contribution_to_firsthit;

    
    if (prd_camera.mode == FIRST_HIT) return;
    // GL Path
    // faceforward(world_shading_normal, -ray.direction, world_geometric_normal);
    float z1 = rnd(prd_camera.seed);
    float z2 = rnd(prd_camera.seed);
    // float z1 = rnd(((unsigned int)(hit_point.x * 100.0f)));
    // float z2 = rnd(((unsigned int)(hit_point.y * 100.0f)));
    float3 p;
    cosine_sample_hemisphere(z1, z2, p);

    optix::Onb onb(forward_normal);
    onb.inverse_transform(p);

    prd_camera.origin = hit_point;
    prd_camera.direction = normalize(p);
    
    // run rendering equation again for this new direction
    float3 contribution_to_this_point = make_float3(0.0f);
    float NdL = dot(forward_normal, prd_camera.direction);
    float3 halfway = normalize(prd_camera.direction - ray.direction);

    float NdH = dot(forward_normal, halfway);
    float VdH = dot(-ray.direction, halfway);  // Same as LdH

    float3 F = make_float3(0.5f);
    // Metallic workflow
    if (false) {
        float3 default_dielectrics_F0 = make_float3(0.04f);
        F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
        subsurface_albedo = (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection
    }
    // Ks Workflow
    else if (true) {
        float3 F0 = Ks * 0.08f;
        F = fresnel_schlick(VdH, 5, F0, make_float3(1) /*make_float3(fresnel_max) it is usually 1*/);
    }
    // Fresnel_at_0 to Fresnel_at_90 workflow
    else if (false) {
        F = fresnel_schlick(VdH, fresnel_exp, make_float3(fresnel_min), make_float3(fresnel_max));
    }
    // IoF workflow
    else {
        // float3 ratio = (iof1 - iof2) / (iof1 + iof2); // one of them should be air (iof = 1)
        // float3 F0 = ratio * ratio;
        // F = fresnel_schlick(NdV, 5, F0, make_float3(1));
    }
    // ==========================================
    // diffuse reflection
    // ==========================================
    contribution_to_this_point += (1 - F) * subsurface_albedo * NdL; 
    // no light fall off or attenuation. Maybe in future when we have volumn we shall change it


    // ==========================================
    // specular reflection
    // ==========================================
    // Get parameters for specular reflection

    float D = NormalDist(NdH, roughness);        // 1/pi omitted
    float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted

    float3 f_ct = F * D * G;

    contribution_to_this_point += f_ct * NdL;
    prd_camera.contribution_to_firsthit *= contribution_to_this_point;

    if (prd_camera.depth == 1) {
        prd_camera.normal = forward_normal;
        prd_camera.albedo = subsurface_albedo;
    }

}
