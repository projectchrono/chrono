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
// RT kernels for material shading
//
// =============================================================================

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"
#include <iostream>

static __device__ __inline__ float3 CrossProduct(const float3& a, const float3& b){

    return {(a.y * b.z) - (b.y * a.z),
            (a.z * b.x) - (b.z * a.x),
            (a.x * b.y) - (b.x * a.y)};
}

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

// triangle mesh querie information
__device__ __inline__ void GetTriangleData(float3& normal,
                                           unsigned int& mat_id,
                                           float2& uv,
                                           float3& tangent,
                                           const unsigned int& mesh_id) {
    const int tri_id = optixGetPrimitiveIndex();
    const float2 bary_coord = optixGetTriangleBarycentrics();

    const MeshParameters& mesh_params = params.mesh_pool[mesh_id];
    const uint4& vertex_idx = mesh_params.vertex_index_buffer[tri_id];

    const float3& v1 = make_float3(mesh_params.vertex_buffer[vertex_idx.x]);
    const float3& v2 = make_float3(mesh_params.vertex_buffer[vertex_idx.y]);
    const float3& v3 = make_float3(mesh_params.vertex_buffer[vertex_idx.z]);

    // calculate normales either from normal buffer or vertex positions
    if (mesh_params.normal_index_buffer &&
        mesh_params.normal_buffer) {  // use vertex normals if normal index buffer exists
        const uint4& normal_idx = mesh_params.normal_index_buffer[tri_id];

        normal = normalize(make_float3(mesh_params.normal_buffer[normal_idx.y]) * bary_coord.x +
                           make_float3(mesh_params.normal_buffer[normal_idx.z]) * bary_coord.y +
                           make_float3(mesh_params.normal_buffer[normal_idx.x]) * (1.0f - bary_coord.x - bary_coord.y));

    } else {  // else use face normals calculated from vertices
        normal = normalize(Cross(v2 - v1, v3 - v1));
    }

    // calculate texcoords if they exist
    if (mesh_params.uv_index_buffer && mesh_params.uv_buffer) {  // use vertex normals if normal index buffer exists
        const uint4& uv_idx = mesh_params.uv_index_buffer[tri_id];
        const float2& uv1 = mesh_params.uv_buffer[uv_idx.x];
        const float2& uv2 = mesh_params.uv_buffer[uv_idx.y];
        const float2& uv3 = mesh_params.uv_buffer[uv_idx.z];

        uv = uv2 * bary_coord.x + uv3 * bary_coord.y + uv1 * (1.0f - bary_coord.x - bary_coord.y);
        float3 e1 = v2 - v1;
        float3 e2 = v3 - v1;
        float2 delta_uv1 = uv2 - uv1;
        float2 delta_uv2 = uv3 - uv1;
        float f = 1.f / (delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y);
        tangent.x = f * (delta_uv2.y * e1.x - delta_uv1.y * e2.x);
        tangent.y = f * (delta_uv2.y * e1.y - delta_uv1.y * e2.y);
        tangent.z = f * (delta_uv2.y * e1.z - delta_uv1.y * e2.z);
        tangent = normalize(tangent);
    } else {
        uv = make_float2(0.f);
        tangent = make_float3(0.f);
    }

    // get material index
    if (mesh_params.mat_index_buffer) {                  // use vertex normals if normal index buffer exists
        mat_id += mesh_params.mat_index_buffer[tri_id];  // the material index gives an offset id
    }
}

//=============================
// Calculating Refracted color
//=============================

static __device__ __inline__ float3 CalculateRefractedColor(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& hit_point,
    const float3& ray_dir){

    float accumulated_transparency = 0.f;
    {
        for (int b = 0; b < num_blended_materials; b++) {
            // accumulate transparency by multiplication
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float mat_opacity = mat.transparency;
            if (mat.opacity_tex) {  // override value with a texture if available
                mat_opacity = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }
            accumulated_transparency += mat_blend_weight * mat_opacity;
        }
    }

    float3 refracted_color = make_float3(0);
    {
        if (accumulated_transparency < 1.f - 1 / 255.f) {
            float3 refract_importance = prd_camera->contrib_to_pixel * (1 - accumulated_transparency);
            if (fmaxf(refract_importance) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
                PerRayData_camera prd_refraction = default_camera_prd();
                prd_refraction.contrib_to_pixel = refract_importance;
                prd_refraction.rng = prd_camera->rng;
                prd_refraction.depth = prd_camera->depth + 1;
                unsigned int opt1, opt2;
                pointer_as_ints(&prd_refraction, opt1, opt2);

                // make_camera_data(make_float3(0), refract_importance, prd_camera.rnd, prd_camera.depth + 1);
                // float3 refract_dir = refract(optixGetWorldRayDirection(), world_normal, 1.f, 1.f);
                float3 refract_dir = ray_dir;  // pure transparency without refraction
                unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
                optixTrace(params.root, hit_point, refract_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                           OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
                refracted_color = prd_refraction.color;  // TODO: not sure added here or not
            }
        }
    }

    return refracted_color;
}

//=====================================================
// Calculating Surface reflection toward light sources
//=====================================================

static __device__ __inline__ float3 CalculateReflectedColor(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& hit_point,
    const float3& world_normal,
    const float3& ray_dir){

    float NdV = Dot(world_normal, -ray_dir);
    float3 light_reflected_color = make_float3(0.0f);
    {
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
                    unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
                    optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light,
                               optixGetRayTime(), OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2,
                               raytype);

                    float3 light_attenuation = prd_shadow.attenuation;

                    float point_light_falloff =
                        (l.max_range * l.max_range / (dist_to_light * dist_to_light + l.max_range * l.max_range));

                    float3 incoming_light_ray = l.color * light_attenuation * point_light_falloff * NdL;

                    if (fmaxf(incoming_light_ray) > 0.0f) {
                        float3 halfway = normalize(dir_to_light - ray_dir);
                        float NdV = Dot(world_normal, -ray_dir);
                        float NdH = Dot(world_normal, halfway);
                        float VdH = Dot(-ray_dir, halfway);

                        for (int b = 0; b < num_blended_materials; b++) {
                            const MaterialParameters& mat = params.material_pool[material_id + b];
                            float3 subsurface_albedo = mat.Kd;
                            if (mat.kd_tex) {
                                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                // transfer sRGB texture into linear color space.
                                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
                            }
                            float roughness = mat.roughness;
                            if (mat.roughness_tex) {
                                roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                            }
                            float metallic = mat.metallic;
                            if (mat.metallic_tex) {
                                metallic = tex2D<float>(mat.metallic_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                            }
                            float transparency = mat.transparency;
                            if (mat.opacity_tex) {  // override value with a texture if available
                                transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                            }
                            float mat_blend_weight = 1.f / num_blended_materials;
                            if (mat.weight_tex) {  // override blending with weight texture if available
                                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
                            }

                            float3 F = make_float3(0.0f);
                            // float3 subsurface_albedo_updated = subsurface_albedo;
                            // === dielectric workflow
                            if (mat.use_specular_workflow) {
                                float3 specular = mat.Ks;
                                if (mat.ks_tex) {
                                    const float4 tex = tex2D<float4>(mat.ks_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                    specular = make_float3(tex.x, tex.y, tex.z);
                                }
                                float3 F0 = specular * 0.08f;
                                F = fresnel_schlick(VdH, 5.f, F0,
                                                    make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
                            } else {
                                float3 default_dielectrics_F0 = make_float3(0.04f);
                                F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                                subsurface_albedo = subsurface_albedo *
                                                    (1 - metallic);  // since imetals do not do subsurface reflection
                            }

                            // Diffuse portion of reflection
                            float3 contrib_weight =
                                prd_camera->contrib_to_pixel * transparency *
                                mat_blend_weight;  // correct for transparency, light bounces, and blend weight
                            light_reflected_color +=
                                ((make_float3(1.f) - F) * subsurface_albedo * incoming_light_ray) * contrib_weight;
                            float D = NormalDist(NdH, roughness);        // 1/pi omitted
                            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
                            float3 f_ct = F * D * G;
                            light_reflected_color += f_ct * incoming_light_ray * contrib_weight;
                        }
                    }
                }
            }
        }

        for(int i = 0; i < params.num_arealights; i++){

            AreaLight a = params.arealights[i];
            
            float dist_to_light = Length(a.pos - hit_point); 
            float3 normal = CrossProduct(a.du, a.dv);
            
            if (dist_to_light < 2 * a.max_range) {
                
                float3 sampleColor = make_float3(0.0f);

                for(int lightSampleID = 0; lightSampleID < 5; lightSampleID++){
                    
                    float3 tempPos = a.pos + (curand_uniform(&prd_camera->rng)*a.du)
                                    + (curand_uniform(&prd_camera->rng)*a.dv);
                    
                    float3 dir_to_light = normalize(tempPos - hit_point);
                    float NdL = Dot(world_normal, dir_to_light);

                    // Dot product of normal of area light and direction to light
                    // float AdL = Dot(a.normal, dir_to_light);

                    // Checking to see if we can hit light rays towards the source and the orientation of the area light
                    // Allows the light ray to hit light-emitting surface part of area light
                    
                    if (NdL > 0.0f) {
                        // check shadows
                        PerRayData_shadow prd_shadow = default_shadow_prd();
                        prd_shadow.depth = prd_camera->depth + 1;
                        prd_shadow.ramaining_dist = dist_to_light;
                        unsigned int opt1;
                        unsigned int opt2;
                        pointer_as_ints(&prd_shadow, opt1, opt2);
                        unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;

                        // TODO: Re-implement this multiple times with slightly different dir_to_light values to improve data sampling

                        optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light,
                                optixGetRayTime(), OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2,
                                raytype);

                        float3 light_attenuation = prd_shadow.attenuation;

                        float point_light_falloff = (a.max_range * a.max_range / (dist_to_light * dist_to_light + a.max_range * a.max_range));
                        float3 incoming_light_ray = a.color * light_attenuation * point_light_falloff * NdL;

                        if (fmaxf(incoming_light_ray) > 0.0f) {

                            float3 halfway = normalize(dir_to_light - ray_dir);
                            float NdV = Dot(world_normal, -ray_dir);
                            float NdH = Dot(world_normal, halfway);
                            float VdH = Dot(-ray_dir, halfway);

                            for (int b = 0; b < num_blended_materials; b++) {
                                const MaterialParameters& mat = params.material_pool[material_id + b];
                                float3 subsurface_albedo = mat.Kd;
                                
                                if (mat.kd_tex) {
                                    const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                    // transfer sRGB texture into linear color space.
                                    subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
                                }

                                float roughness = mat.roughness;
                                if (mat.roughness_tex) {
                                    roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                }
                                float metallic = mat.metallic;
                                if (mat.metallic_tex) {
                                    metallic = tex2D<float>(mat.metallic_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                }
                                float transparency = mat.transparency;
                                if (mat.opacity_tex) {  // override value with a texture if available
                                    transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                }
                                float mat_blend_weight = 1.f / num_blended_materials;
                                if (mat.weight_tex) {  // override blending with weight texture if available
                                    mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
                                }

                                float3 F = make_float3(0.0f);
                                // float3 subsurface_albedo_updated = subsurface_albedo;
                                // === dielectric workflow
                                if (mat.use_specular_workflow) {
                                    float3 specular = mat.Ks;
                                    if (mat.ks_tex) {
                                        const float4 tex = tex2D<float4>(mat.ks_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                                        specular = make_float3(tex.x, tex.y, tex.z);
                                    }
                                    float3 F0 = specular * 0.08f;
                                    F = fresnel_schlick(VdH, 5.f, F0,
                                                        make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
                                } else {
                                    float3 default_dielectrics_F0 = make_float3(0.04f);
                                    F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                                    subsurface_albedo = subsurface_albedo *
                                                        (1 - metallic);  // since imetals do not do subsurface reflection
                                }

                                // Diffuse portion of reflection
                                float3 contrib_weight =
                                    prd_camera->contrib_to_pixel * transparency *
                                    mat_blend_weight;  // correct for transparency, light bounces, and blend weight
                                sampleColor +=
                                    ((make_float3(1.f) - F) * subsurface_albedo * incoming_light_ray) * contrib_weight;
                                float D = NormalDist(NdH, roughness);        // 1/pi omitted
                                float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted

                                float3 f_ct = F * D * G;
                                sampleColor += f_ct * incoming_light_ray * contrib_weight;
                            }
                        }
                    }
                }

                sampleColor = sampleColor / 5;
                light_reflected_color += sampleColor;
            }
        }
    }

    return light_reflected_color;
}

//===========================
// Calculating Ambient Light
//===========================

static __device__ __inline__ float3 CalculateAmbientLight(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& world_normal,
    const float3& ray_dir){

    float3 ambient_light = make_float3(0.0f);
    {
        if (!prd_camera->use_gi) {
            float NdV = Dot(world_normal, -ray_dir);
            for (int b = 0; b < num_blended_materials; b++) {
                const MaterialParameters& mat = params.material_pool[material_id + b];
                float3 subsurface_albedo = mat.Kd;
                if (mat.kd_tex) {
                    const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                    // transfer sRGB texture into linear color space.
                    subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
                }
                float transparency = mat.transparency;
                if (mat.opacity_tex) {  // override value with a texture if available
                    transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                }
                float mat_blend_weight = 1.f / num_blended_materials;
                if (mat.weight_tex) {  // override blending with weight texture if available
                    mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
                }

                float3 contrib_weight = prd_camera->contrib_to_pixel * transparency *
                                       mat_blend_weight;  // correct for transparency, light bounces, and blend weight

                // ambient light model is partial "flashlight" ambient light, partially from normal direction
                ambient_light += params.ambient_light_color *
                                 (make_float3(NdV) + make_float3(Dot(world_normal, make_float3(0, 0, 1)) * .5f + .5f)) *
                                 subsurface_albedo * contrib_weight;
            }
        }
    }

    return ambient_light;
}

//===============================================================
// If the surface is very smoooth, trace the reflected direction
// Do this reflection regardless of GI on or off.
//===============================================================

static __device__ __inline__ float3 CalculateContributionToPixel(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& world_normal,
    const float3& ray_dir,
    const float3& hit_point){
    
    float NdV = Dot(world_normal, -ray_dir);
    float3 next_contrib_to_pixel = make_float3(0.f);
    float3 next_dir = normalize(reflect(ray_dir, world_normal));
    {
        float NdL = Dot(world_normal, next_dir);
        float3 halfway = normalize(next_dir - ray_dir);
        float NdH = Dot(world_normal, halfway);
        float VdH = Dot(-ray_dir, halfway);  // Same as LdH

        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float3 subsurface_albedo = mat.Kd;
            if (mat.kd_tex) {
                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x, uv.y);
                // transfer sRGB texture into linear color space.
                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
            }
            float roughness = mat.roughness;
            if (mat.roughness_tex) {
                roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float metallic = mat.metallic;
            if (mat.metallic_tex) {
                metallic = tex2D<float>(mat.metallic_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float transparency = mat.transparency;
            if (mat.opacity_tex) {  // override value with a texture if available
                transparency = tex2D<float>(mat.opacity_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }

            float3 F = make_float3(0.0f);
            // === dielectric workflow
            if (mat.use_specular_workflow) {
                float3 specular = mat.Ks;
                if (mat.ks_tex) {
                    const float4 tex = tex2D<float4>(mat.ks_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                    specular = make_float3(tex.x, tex.y, tex.z);
                }
                float3 F0 = specular * 0.08f;
                F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
            } else {
                float3 default_dielectrics_F0 = make_float3(0.04f);
                F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
            }

            float D = NormalDist(NdH, roughness);        // 1/pi omitted
            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted

            float3 f_ct = F * D * G;

            // Note only specular part appears here. Energy preserve
            // Since it is not random, PDF is 1 (normally 1/pi),
            // If the camera uses GI, then it will trace two rays. So each ray's contribution should be halfed

            // corrected for transparency, bounce contribution, and blend
            float weight = transparency * mat_blend_weight;

            // mirror correction accounts for us oversampling this direction
            // following line comes from a heuristic. Perect reflection for metalic smooth objects,
            // no reflection for rough non-metalic objects
            float mirror_correction = (1.f - roughness) * (1.f - roughness) * metallic * metallic;

            // if global illumination, ray contrib will be halved since two rays are propogated
            if (prd_camera->use_gi) {
                weight = weight*.5f;
            }

            float3 partial_contrib = mirror_correction * weight * f_ct * NdL / (4 * CUDART_PI_F);
            partial_contrib = clamp(partial_contrib, make_float3(0), make_float3(1));

            partial_contrib = partial_contrib * prd_camera->contrib_to_pixel;

            next_contrib_to_pixel += partial_contrib;
            next_contrib_to_pixel = clamp(next_contrib_to_pixel, make_float3(0), make_float3(1));
        }
    }

    float3 mirror_reflection_color = make_float3(0.0);
    {
        if (luminance(next_contrib_to_pixel) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_reflection = default_camera_prd();
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;

            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
            optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

            mirror_reflection_color = prd_reflection.color;
        }
    }

    return mirror_reflection_color;
}

//=================
// Global illumination ray
//=================

static __device__ __inline__ float3 CalculateGIReflectionColor(
    PerRayData_camera* prd_camera,
    const int& num_blended_materials,
    unsigned int& material_id,
    const float2& uv,
    const float3& world_normal,
    const float3& ray_dir,
    const float3& hit_point,
    const float3& mirror_reflection_color){

    float NdV = Dot(world_normal, -ray_dir);
    float3 gi_reflection_color = make_float3(0);

    if (prd_camera->use_gi) {
        // sample hemisphere for next ray when using global illumination
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        float3 next_dir = sample_hemisphere_dir(z1, z2, world_normal);

        float NdL = Dot(world_normal, next_dir);
        float3 halfway = normalize(next_dir - ray_dir);

        float NdH = Dot(world_normal, halfway);
        float VdH = Dot(-ray_dir, halfway);  // Same as LdH

        float3 next_contrib_to_pixel = make_float3(0.f);

        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float3 subsurface_albedo = mat.Kd;
            if (mat.kd_tex) {
                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                // transfer sRGB texture into linear color space.
                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
            }
            float roughness = mat.roughness;
            if (mat.roughness_tex) {
                roughness = tex2D<float>(mat.roughness_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float metallic = mat.metallic;
            if (mat.metallic_tex) {
                metallic = tex2D<float>(mat.metallic_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float transparency = mat.transparency;
            if (mat.opacity_tex) {  // override value with a texture if available
                transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }

            float3 F = make_float3(0.0f);
            // === dielectric workflow
            if (mat.use_specular_workflow) {
                float3 specular = mat.Ks;
                if (mat.ks_tex) {
                    const float4 tex = tex2D<float4>(mat.ks_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                    specular = make_float3(tex.x, tex.y, tex.z);
                }
                float3 F0 = specular * 0.08f;
                F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
            } else {
                float3 default_dielectrics_F0 = make_float3(0.04f);
                F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                subsurface_albedo = subsurface_albedo*(1 - metallic);  // since metals do not do subsurface reflection
            }

            float D = NormalDist(NdH, roughness);        // 1/pi omitted
            float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
            float3 f_ct = F * D * G;

            // corrected for transparency, bounce contribution, and blend
            float3 weight = transparency * prd_camera->contrib_to_pixel * mat_blend_weight;

            // If mirror_reflection, then it will trace two rays. So each ray's contribution should be halfed
            if ((mirror_reflection_color.x < 1e-6) && (mirror_reflection_color.y < 1e-6) && (mirror_reflection_color.z < 1e-6)) {
                weight = weight*.5f;
            }

            // Specular part
            next_contrib_to_pixel += weight * f_ct * NdL;

            // Diffuse part
            F = clamp(F, make_float3(0), make_float3(1));
            next_contrib_to_pixel += weight * (make_float3(1.f) - F) * subsurface_albedo * NdL;
        }

        if (luminance(next_contrib_to_pixel) > params.importance_cutoff &&
            prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_reflection = default_camera_prd();
            prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
            prd_reflection.rng = prd_camera->rng;
            prd_reflection.depth = prd_camera->depth + 1;
            prd_reflection.use_gi = prd_camera->use_gi;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_reflection, opt1, opt2);
            unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
            optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                        OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            gi_reflection_color = prd_reflection.color;  // accumulate indirect lighting color
        }
    }

    return gi_reflection_color;
}

static __device__ __inline__ void CameraShader(PerRayData_camera* prd_camera,
                                               const MaterialRecordParameters* mat_params,
                                               unsigned int& material_id,
                                               const unsigned int& num_blended_materials,
                                               const float3& world_normal,
                                               const float2& uv,
                                               const float3& tangent,
                                               const float& ray_dist,
                                               const float3& ray_orig,
                                               const float3& ray_dir) {
    float3 hit_point = ray_orig + ray_dir * ray_dist;

    // if not blended materials, check for transparent cards and short circuit on the transparent texture
    const MaterialParameters& mat = params.material_pool[material_id];
    if (num_blended_materials == 1) {
       

        float transparency = mat.transparency;
        // figure out tranparency
        if (mat.kd_tex) {
            const float4 tex = tex2D<float4>(mat.kd_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
            if (tex.w < 1e-6)
                transparency = 0.f;  // to handle transparent card textures such as tree leaves
        }

        if (mat.opacity_tex) {
            transparency = tex2D<float>(mat.opacity_tex,uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
        }

        // if this is perfectly transparent, we ignore it and trace the next ray (handles things like tree leaf cards)
        if (transparency < 1e-6) {
            if (prd_camera->depth + 1 < params.max_depth) {
                PerRayData_camera prd_refraction = default_camera_prd();
                prd_refraction.contrib_to_pixel = prd_camera->contrib_to_pixel;
                prd_refraction.rng = prd_camera->rng;
                prd_refraction.depth = prd_camera->depth + 1;
                unsigned int opt1, opt2;
                pointer_as_ints(&prd_refraction, opt1, opt2);
                unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
                optixTrace(params.root, hit_point, ray_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                           OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
                prd_camera->color = prd_refraction.color;
                //account for fog
                if (prd_camera->use_fog && params.fog_scattering > 0.f) {
                    float blend_alpha = expf(-params.fog_scattering * ray_dist);
                    prd_camera->color = blend_alpha * prd_camera->color + (1 - blend_alpha) * params.fog_color;
                }

                // For GI, harmless without GI
                prd_camera->albedo = prd_refraction.albedo;
                prd_camera->normal = prd_refraction.normal;
            }
            return;
        }
    }

    // for each blended material accumulate transparency, and perform traversal
    float3 refracted_color = CalculateRefractedColor(prd_camera, num_blended_materials, material_id, uv, hit_point, ray_dir);

    // for each light, traverse to light, and calculate each material's shading
    float3 light_reflected_color = CalculateReflectedColor(prd_camera, num_blended_materials, material_id, uv, hit_point, world_normal, ray_dir);

    // for each blended material, calculating total ambient light
    float3 ambient_light = CalculateAmbientLight(prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir);

    // for each blended material accumulate reflection, and perform traversal    
    float3 next_dir = normalize(reflect(ray_dir, world_normal));
    float3 mirror_reflection_color = CalculateContributionToPixel(prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir, hit_point);

    // send ray in random direction if global illumination enabled, calculate each materia's shading for a combined shading
    float3 gi_reflection_color = CalculateGIReflectionColor(prd_camera, num_blended_materials, material_id, uv, world_normal, ray_dir, hit_point, mirror_reflection_color);
    
    //=================
    // Combine all traced colors together
    //=================
    prd_camera->color = mirror_reflection_color + light_reflected_color + refracted_color;

    prd_camera->color += prd_camera->use_gi ? gi_reflection_color : ambient_light;

    // Add emissive color
    prd_camera->color += (mat.emissive_power * mat.Ke * abs(Dot(world_normal, -ray_dir)));
    

    // apply fog model
    if (prd_camera->use_fog && params.fog_scattering > 0.f) {
        float blend_alpha = expf(-params.fog_scattering * ray_dist);
        prd_camera->color = blend_alpha * prd_camera->color + (1 - blend_alpha) * params.fog_color;
    }

    //printf("Color: (%.2f,%.2f,%.2f)\n", prd_camera->color.x, prd_camera->color.y, prd_camera->color.z);
    if (prd_camera->depth == 2 && prd_camera->use_gi) {
        float3 accumulated_subsurface_albedo = make_float3(0.f);
        for (int b = 0; b < num_blended_materials; b++) {
            const MaterialParameters& mat = params.material_pool[material_id + b];
            float3 subsurface_albedo = mat.Kd;
            if (mat.kd_tex) {
                const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
                // transfer sRGB texture into linear color space.
                subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
            }
            float mat_blend_weight = 1.f / num_blended_materials;
            if (mat.weight_tex) {  // override blending with weight texture if available
                mat_blend_weight = tex2D<float>(mat.weight_tex, uv.x, uv.y);
            }
            accumulated_subsurface_albedo += subsurface_albedo * mat_blend_weight;
        }
        prd_camera->albedo = accumulated_subsurface_albedo;
        prd_camera->normal = world_normal;
    }

    
}

static __device__ __inline__ void LidarShader(PerRayData_lidar* prd_lidar,
                                              const MaterialParameters& mat,
                                              const float3& world_normal,
                                              const float2& uv,
                                              const float3& tangent,
                                              const float& ray_dist,
                                              const float3& ray_orig,
                                              const float3& ray_dir) {
    prd_lidar->range = ray_dist;
    prd_lidar->intensity = mat.lidar_intensity * abs(Dot(world_normal, -ray_dir));
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

static __device__ __inline__ void ShadowShader(PerRayData_shadow* prd,
                                               const MaterialParameters& mat,
                                               const float3& world_normal,
                                               const float2& uv,
                                               const float3& tangent,
                                               const float& ray_dist,
                                               const float3& ray_orig,
                                               const float3& ray_dir) {
    float transparency = mat.transparency;
    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
        if (tex.w < 1e-6)
            transparency = 0.f;  // to handle transparent card textures such as tree leaves
    }
    if (mat.opacity_tex) {
        transparency = tex2D<float>(mat.opacity_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
    }
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float atten = 1.f - transparency;  // TODO: figure out the attenuation from the material transparency

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
        unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
        optixTrace(params.root, hit_point, ray_dir, params.scene_epsilon, prd_shadow.ramaining_dist, optixGetRayTime(),
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

        prd->attenuation = prd_shadow.attenuation;
    }
}

static __device__ __inline__ void SemanticShader(PerRayData_semantic* prd,
                                                 const MaterialParameters& mat,
                                                 const float3& world_normal,
                                                 const float2& uv,
                                                 const float3& tangent,
                                                 const float& ray_dist,
                                                 const float3& ray_orig,
                                                 const float3& ray_dir) {
    prd->class_id = mat.class_id;
    prd->instance_id = mat.instance_id;
}

static __device__ inline void CameraHapkeShader(PerRayData_camera* prd_camera,
                                               const MaterialRecordParameters* mat_params,
                                               unsigned int& material_id,
                                               const unsigned int& num_blended_materials,
                                               const float3& world_normal,
                                               const float2& uv,
                                               const float3& tangent,
                                               const float& ray_dist,
                                               const float3& ray_orig,
                                               const float3& ray_dir){
        //printf("Hapke Shader!\n");
        float w = 0.32357f; // average single scattering albedo
        float b = 0.23955f; // shape controlling parameter for the amplitude of backward and forward scatter of particles
        float c = 0.30452f; // weighting factor that controls the contribution of backward and forward scatter.
        float B_s0 = 1.80238f;
        float h_s = 0.07145f;
        float B_c0 = 0.0f;
        float h_c = 1.0f;
        float phi = 0.3f;
        //float K = 1.0f;
        float theta_p = 23.4f*(CUDART_PI_F/180);

        const MaterialParameters& mat = params.material_pool[material_id]; // Assume no blended materials for now
        float3 subsarface_albedo = mat.Kd;
        float3 specular = mat.Ks;

        float3 hit_point = ray_orig + ray_dir * ray_dist;

        float cos_e = Dot(world_normal, -ray_dir);

        float3 reflected_color = make_float3(0.0f);
        {
            for (int i = 0; i < params.num_lights; i++) {
                PointLight l = params.lights[i];
                float dist_to_light = Length(l.pos - hit_point);
                //printf("dist_to_light:%.4f\n", dist_to_light);
                if (1) {//dist_to_light < 2 * l.max_range{ // Sun should have infinity range, so this condition will always be true for ths sun
                    float3 dir_to_light = normalize(l.pos - hit_point);
                    float cos_i = Dot(dir_to_light, world_normal);
                    //printf("cos_i:%.2f",cos_i);
                    if (cos_i > 0) {

                        // Cast a shadow ray to see any attenuation of light
                        PerRayData_shadow prd_shadow = default_shadow_prd();
                        prd_shadow.depth = prd_camera->depth + 1;
                        prd_shadow.ramaining_dist = dist_to_light;
                        unsigned int opt1;
                        unsigned int opt2;
                        pointer_as_ints(&prd_shadow, opt1, opt2);
                        unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
                        optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light, optixGetRayTime(),
                                OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

                        float3 light_attenuation = prd_shadow.attenuation;

                        float point_light_falloff  = 1.0f; // ??
                        float3 incoming_light_ray = l.color * cos_i * light_attenuation; // Add attenuation later
                        //printf("incoming_light_ray: (%.2f,%.2f,%.2f)\n", incoming_light_ray.x, incoming_light_ray.y, incoming_light_ray.z);
                        if (fmaxf(incoming_light_ray) > 0.0f) {
                            
                            float cos_g = Dot(dir_to_light, -ray_dir);
                            float sin_i = sqrt(1 - (cos_i*cos_i)); // + sqrt
                            float sin_e = sqrt(1 - (cos_e*cos_e));
                            float sin_g = sqrt(1 - (cos_g*cos_g));

                            float tan_i = sin_i/cos_i;
                            float tan_e = sin_e/cos_e;

                            float cot_i = 1/tan_i;
                            float cot_e = 1/tan_e;
                            float cot_i_sq = cot_i * cot_i;
                            float cot_e_sq = cot_e * cot_e;


                            // Calculate Psi
                            float cos_psi = Dot(normalize(dir_to_light - (cos_i*world_normal)), normalize(-ray_dir - (cos_e*world_normal)));
                            float psi = acos(cos_psi);
                            float psi_half = psi/2;
                            float f_psi = expf(-2 * tan(psi_half));
                            float sin_psi_half = sin(psi_half);
                            float sin_psi_half_sq = sin_psi_half * sin_psi_half;
                            float psi_per_pi = psi/CUDART_PI_F; // TODO: Define 1/PI as a constant

                            float tan_theta_p = tan(theta_p);
                            float tan_theta_p_sq = tan_theta_p * tan_theta_p;

                            float cot_theta_p = 1/tan_theta_p;
                            float cot_theta_p_sq = cot_theta_p * cot_theta_p;

                            float E_1_i = expf((-2/CUDART_PI_F) * cot_theta_p * cot_i);
                            float E_2_i = expf((-1/CUDART_PI_F) * cot_theta_p * cot_theta_p * cot_i * cot_i);

                            float E_1_e = expf((-2/CUDART_PI_F) * cot_theta_p * cot_e);
                            float E_2_e = expf((-1/CUDART_PI_F) * cot_theta_p * cot_theta_p * cot_e * cot_e);

                            float chi_theta_p = 1 / sqrtf(1 + CUDART_PI_F * tan_theta_p_sq);
                            
                            float eta_i = chi_theta_p * (cos_i + sin_i * tan_theta_p * E_2_i / (2 - E_1_i));
                            float eta_e = chi_theta_p * (cos_e + sin_e * tan_theta_p * E_2_e / (2 - E_1_e));

                            float mu0 = cos_i;
                            float mu = cos_e;
                            float mu0_e = chi_theta_p;
                            float mu_e = chi_theta_p;
                            float S = 0.0f;
                            if (cos_i >= cos_g) { // for x,y \in [0,pi], if x <= y => cos(x) >= cos(y)
                                mu0_e *= cos_i + sin_i * tan_theta_p * (cos_psi * E_2_e + sin_psi_half_sq * E_2_i) / (2 - E_1_e - psi_per_pi * E_1_i);

                                mu_e *= cos_e + sin_e * tan_theta_p * (E_2_e - sin_psi_half_sq * E_2_i) / (2 - E_1_e - psi_per_pi * E_1_i);

                                S = mu_e / eta_e * mu0 / eta_i * chi_theta_p / (1 - f_psi + f_psi * chi_theta_p * (mu0/eta_i));
                            }else {
                                mu0_e *= cos_i + sin_i * tan_theta_p * (E_2_i - sin_psi_half_sq * E_2_e) / (2 - E_1_i - psi_per_pi * E_1_e);

                                mu_e *= cos_e + sin_e * tan_theta_p * (cos_psi * E_2_i + sin_psi_half_sq * E_2_e) / (2 - E_1_i - psi_per_pi * E_1_e);

                                S = mu_e / eta_e * mu0 / eta_i * chi_theta_p / (1 - f_psi + f_psi * chi_theta_p * (mu/eta_e));
                            }

                            float KPhi = 1.209 * pow(phi, 2.0f/3);
                            float K = -log(1 - KPhi)/KPhi;

                            float tan_ghalf = sin_g / (1 + cos_g);
                            float tan_ghalf_per_hC = tan_ghalf/h_c;

                            float B_C = 0.0f;
                            if (cos_g < 1.0f)
                                B_C = (1 + (1 - exp(-tan_ghalf_per_hC)) / tan_ghalf_per_hC) / (2 * pow(1 + tan_ghalf_per_hC, 2));
                            else if (cos_g == 1)
                                B_C = 1;
                            
                            float r0Term = sqrt(1 - w);
                            float r0 = (1 - r0Term)/(1 + r0Term);

                            float LS = mu0_e / (mu0_e + mu_e);
                            float b_sq = b*b;

                            float twobcos_g = 2 * b * cos_g;

                            float p_g = (1 + c) / 2 * (1-b_sq) / pow(1 - (2*b*cos_g) + b_sq, 1.5f) + (1 - c)/2 * (1-b_sq)/pow(1 + (2*b*cos_g) + b_sq, 1.5f);

                            float B_S = 1 / (1 + tan_ghalf / h_s);

                            float x_i = mu0_e/K;
                            float x_e = mu_e/K;
                            float H_i = 1/(1 - w * x_i * (r0 + (1 - 2 * r0 * x_i) / 2 * log((1+x_i)/x_i)));
                            float H_e = 1/(1 - w * x_e * (r0 + (1 - 2 * r0 * x_e) / 2 * log((1+x_e)/x_e)));

                            float M = H_i * H_e - 1;
                            float f_ct = LS * K * w/(4*CUDART_PI_F) * (p_g * (1 + B_s0 * B_S) + M) * (1 + B_c0 * B_C) * S /cos_i;
                            //printf("fct:%.2f\n", f_ct);
                            reflected_color += f_ct * incoming_light_ray * subsarface_albedo;
                            //printf("reflected_color:(%.2f,%.2f,%.2f)\n", reflected_color.x, reflected_color.y, reflected_color.z);
                        }

                    }       
                }
            }
        }
     //printf("reflected_color:(%.2f,%.2f,%.2f)\n", reflected_color.x, reflected_color.y, reflected_color.z);
     prd_camera->color += reflected_color;
}

static __device__ __inline__ void DepthShader(PerRayData_depthCamera* prd,
                                                 const MaterialParameters& mat,
                                                 const float3& world_normal,
                                                 const float2& uv,
                                                 const float3& tangent,
                                                 const float& ray_dist,
                                                 const float3& ray_orig,
                                                 const float3& ray_dir) {
    prd->depth = ray_dist;
}

extern "C" __global__ void __closesthit__material_shader() {
    // determine parameters that are shared across all ray types
    const MaterialRecordParameters* mat_params = (MaterialRecordParameters*)optixGetSbtDataPointer();

    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir = normalize(optixGetWorldRayDirection());  // this may be modified by the scaling transform
    const float ray_dist = optixGetRayTmax();

    float3 object_normal;
    float2 uv;
    float3 tangent;

    // check if we hit a triangle
    unsigned int material_id = mat_params->material_pool_id;
    if (optixIsTriangleHit()) {
        GetTriangleData(object_normal, material_id, uv, tangent, mat_params->mesh_pool_id);
    } else {
        object_normal = make_float3(__int_as_float(optixGetAttribute_0()), __int_as_float(optixGetAttribute_1()),
                                    __int_as_float(optixGetAttribute_2()));
        uv = make_float2(__int_as_float(optixGetAttribute_3()), __int_as_float(optixGetAttribute_4()));
        tangent = make_float3(__int_as_float(optixGetAttribute_5()), __int_as_float(optixGetAttribute_6()),
                              __int_as_float(optixGetAttribute_7()));
    }

    const MaterialParameters& mat = params.material_pool[material_id];

    if (mat.kn_tex) {
        float3 bitangent = normalize(Cross(object_normal, tangent));
        const float4 tex = tex2D<float4>(mat.kn_tex, uv.x*mat.tex_scale.x, uv.y*mat.tex_scale.y);
        float3 normal_delta = make_float3(tex.x, tex.y, tex.z) * 2.f - make_float3(1.f);
        object_normal =
            normalize(normal_delta.x * tangent + normal_delta.y * bitangent + normal_delta.z * object_normal);
    }

    float3 world_normal = normalize(optixTransformNormalFromObjectToWorldSpace(object_normal));

    // from here on out, things are specific to the ray type
    RayType raytype = (RayType)optixGetPayload_2();

    switch (raytype) {
        case CAMERA_RAY_TYPE:
            // CameraShader(getCameraPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir,
            // mat_params->num_blended_materials);
            //printf("Use Hapke: %d | Emissive Power: %.2f | Material ID: %d\n", mat.use_hapke, mat.emissive_power, material_id);
            if (!mat.use_hapke) {
                CameraShader(getCameraPRD(), mat_params, material_id, mat_params->num_blended_materials, world_normal, uv,
                            tangent, ray_dist, ray_orig, ray_dir);
            }else {
                CameraHapkeShader(getCameraPRD(), mat_params, material_id, mat_params->num_blended_materials, world_normal, uv,
                            tangent, ray_dist, ray_orig, ray_dir);
            }
            break;
        case LIDAR_RAY_TYPE:
            LidarShader(getLidarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case RADAR_RAY_TYPE:
            RadarShader(getRadarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir,
                        mat_params->translational_velocity, mat_params->angular_velocity, mat_params->objectId);
            break;
        case SHADOW_RAY_TYPE:
            ShadowShader(getShadowPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case SEGMENTATION_RAY_TYPE:
            SemanticShader(getSemanticPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case DEPTH_RAY_TYPE:
            DepthShader(getDepthCameraPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
    }
}
