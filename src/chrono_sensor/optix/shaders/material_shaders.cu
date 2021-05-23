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

static __device__ __inline__ void CameraShader(PerRayData_camera* prd_camera,
                                               const MaterialParameters& mat,
                                               const float3& world_normal,
                                               const float2& uv,
                                               const float3& tangent,
                                               const float& ray_dist,
                                               const float3& ray_orig,
                                               const float3& ray_dir) {
    float3 subsurface_albedo = mat.Kd;
    float transparency = mat.transparency;
    float3 specular = mat.Ks;
    int use_specular_workfloat = mat.use_specular_workfloat;

    float3 hit_point = ray_orig + ray_dir * ray_dist;

    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x, uv.y);
        subsurface_albedo = make_float3(tex.x, tex.y, tex.z);
        if (tex.w < 1e-6)
            transparency = 0.f;  // to handle transparent card textures such as tree leaves
    }

    if (mat.ks_tex) {
        const float4 tex = tex2D<float4>(mat.ks_tex, uv.x, uv.y);
        specular = make_float3(tex.x, tex.y, tex.z);
    }

    if (mat.opacity_tex) {
        transparency = tex2D<float>(mat.opacity_tex, uv.x, uv.y);
    }

    // if this is perfectly transparent, we ignore it and trace the next ray (handles things like tree leaf cards)
    if (transparency < 1e-6) {
        float refract_importance = prd_camera->contrib_to_first_hit;
        if (refract_importance > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_refraction = default_camera_prd();
            prd_refraction.contrib_to_first_hit = refract_importance;
            prd_refraction.rng = prd_camera->rng;
            prd_refraction.depth = prd_camera->depth + 1;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_refraction, opt1, opt2);
            unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
            optixTrace(params.root, hit_point, ray_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            prd_camera->color = prd_refraction.color;
        }
        return;
    }

    //=================
    // Refracted color
    //=================
    float3 refracted_color = make_float3(0);
    if (transparency < 0.99f) {
        float refract_importance = prd_camera->contrib_to_first_hit * (1 - transparency);
        if (refract_importance > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_refraction = default_camera_prd();
            prd_refraction.contrib_to_first_hit = refract_importance;
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
                unsigned int raytype = (unsigned int)SHADOW_RAY_TYPE;
                optixTrace(params.root, hit_point, dir_to_light, params.scene_epsilon, dist_to_light, optixGetRayTime(),
                           OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);

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
                    if (use_specular_workfloat) {
                        float3 F0 = specular * 0.08f;
                        F = fresnel_schlick(VdH, 5.f, F0,
                                            make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);

                    } else {
                        float3 default_dielectrics_F0 = make_float3(0.04f);
                        F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                        subsurface_albedo =
                            (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection
                    }                                            // default to specular workflow

                    diffuse_color += (make_float3(1.f) - F) * subsurface_albedo * incoming_light_ray;
                    float D = NormalDist(NdH, roughness);        // 1/pi omitted
                    float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
                    float3 f_ct = F * D * G;
                    diffuse_color += f_ct * incoming_light_ray;
                }
            }
        }
    }

    float NdV = Dot(world_normal, -ray_dir);
    // diffuse_color = diffuse_color + params.ambient_light_color * make_float3(NdV) * subsurface_albedo;
    // diffuse_color = diffuse_color + params.ambient_light_color *
    //                                     make_float3(Dot(world_normal, make_float3(0, 0, 1)) * .5f + .5f) *
    //                                     subsurface_albedo;
    diffuse_color =
        diffuse_color + params.ambient_light_color *
                            (make_float3(NdV) + make_float3(Dot(world_normal, make_float3(0, 0, 1)) * .5f + .5f)) *
                            subsurface_albedo;

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
    float3 F = make_float3(0.5f);
    // === dielectric workflow
    if (metallic > 0) {
        float3 default_dielectrics_F0 = make_float3(0.04f);
        F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
        subsurface_albedo = (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection
    } else {                                                     // default to specular workflow
        float3 F0 = mat.Ks * 0.08f;
        F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
    }

    float3 next_contrib_to_first_hit = (make_float3(1.f) - F) * subsurface_albedo * NdL;

    float D = NormalDist(NdH, roughness);        // 1/pi omitted
    float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
    float3 f_ct = F * D * G;

    next_contrib_to_first_hit += f_ct * NdL;

    // for whitted ray tracing, add reflection based on how smooth the material is (non-physical, sharp reflections)
    if (!prd_camera->use_gi) {  // we need to account for the fact we didn't randomly sample the direction (heuristic
        // since this is not physical)
        next_contrib_to_first_hit =
            clamp(next_contrib_to_first_hit / (CUDART_PI_F * 4), make_float3(0.f), make_float3(1.f)) *
            prd_camera->contrib_to_first_hit;

        next_contrib_to_first_hit =
            (1.f - roughness) * (1.f - roughness) * metallic * metallic * next_contrib_to_first_hit;
    } else {
        next_contrib_to_first_hit = next_contrib_to_first_hit * prd_camera->contrib_to_first_hit;
    }

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
        unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
        optixTrace(params.root, hit_point, next_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                   OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
        prd_camera->color += prd_reflection.color;  // accumulate indirect lighting color
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
                                              const float& objectID) {
    prd_radar->range = ray_dist;
    prd_radar->rcs = mat.radar_backscatter * abs(Dot(world_normal, -ray_dir));
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float3 origin = optixTransformPointFromObjectToWorldSpace(make_float3(0,0,0));
    float3 r = hit_point - origin;
    
    prd_radar->velocity = translational_velocity +  Cross(angular_velocity , r);
    prd_radar->objectID = objectID;
//    float3 linear = Cross(angular_velocity, r);
//    printf("%f %f %f\n", r.x, r.y, r.z);
//    printf("%f %f %f\n", angular_velocity.x, angular_velocity.y, angular_velocity.z);
//    printf("%f %f %f\n", linear.x, linear.y, linear.z);
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
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x, uv.y);
        if (tex.w < 1e-6)
            transparency = 0.f;  // to handle transparent card textures such as tree leaves
    }
    if (mat.opacity_tex) {
        transparency = tex2D<float>(mat.opacity_tex, uv.x, uv.y);
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

extern "C" __global__ void __closesthit__material_shader() {
    // determine parameters that are shared across all ray types
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
    
    // radar 
    float3 angular_velocity = mat_params->angular_velocity;
    float3 translational_velocity = mat_params->translational_velocity;

//    printf("translational velocity: %f %f\n", mat_params->translational_velocity.x, mat_params->translational_velocity.y);

    // from here on out, things are specific to the ray type
    RayType raytype = (RayType)optixGetPayload_2();

    switch (raytype) {
        case CAMERA_RAY_TYPE:
            CameraShader(getCameraPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case LIDAR_RAY_TYPE:
            LidarShader(getLidarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case RADAR_RAY_TYPE:
            RadarShader(getRadarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir, translational_velocity, angular_velocity, mat_params->objectID);
            break;
        case SHADOW_RAY_TYPE:
            ShadowShader(getShadowPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
    }
}
