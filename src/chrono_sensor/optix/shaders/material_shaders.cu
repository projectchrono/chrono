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
    // Colors in MTL files are already in linear color space
    float3 subsurface_albedo = mat.Kd;
    float transparency = mat.transparency;
    float3 specular = mat.Ks;
    int use_specular_workflow = mat.use_specular_workflow;
    float3 hit_point = ray_orig + ray_dir * ray_dist;
    float NdV = Dot(world_normal, -ray_dir);
    if (mat.kd_tex) {
        const float4 tex = tex2D<float4>(mat.kd_tex, uv.x, uv.y);
        // transfer sRGB texture into linear color space. All of them need this transform
        subsurface_albedo = Pow(make_float3(tex.x, tex.y, tex.z), 2.2);
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
        float3 refract_importance = prd_camera->contrib_to_pixel;
        if (fmaxf(refract_importance) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
            PerRayData_camera prd_refraction = default_camera_prd();
            prd_refraction.contrib_to_pixel = refract_importance;
            prd_refraction.rng = prd_camera->rng;
            prd_refraction.depth = prd_camera->depth + 1;
            unsigned int opt1, opt2;
            pointer_as_ints(&prd_refraction, opt1, opt2);
            unsigned int raytype = (unsigned int)CAMERA_RAY_TYPE;
            optixTrace(params.root, hit_point, ray_dir, params.scene_epsilon, 1e16f, optixGetRayTime(),
                       OptixVisibilityMask(1), OPTIX_RAY_FLAG_NONE, 0, 1, 0, opt1, opt2, raytype);
            prd_camera->color = prd_refraction.color;
            // For GI, harmless without GI
            prd_camera->albedo = prd_refraction.albedo;
            prd_camera->normal = prd_refraction.normal;
        }
        return;
    }

    //=================
    // Refracted color
    //=================
    float3 refracted_color = make_float3(0);
    if (transparency < 0.99f) {
        float3 refract_importance = prd_camera->contrib_to_pixel * (1 - transparency);
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
    // Surface reflection toward light sources
    //=================
    float3 reflected_color = make_float3(0.0f);
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

                    float3 F = make_float3(0.0f);
                    float3 subsurface_albedo_updated = subsurface_albedo;
                    // === dielectric workflow
                    if (use_specular_workflow) {
                        float3 F0 = specular * 0.08f;
                        F = fresnel_schlick(VdH, 5.f, F0,
                                            make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
                    } else {
                        float3 default_dielectrics_F0 = make_float3(0.04f);
                        F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
                        subsurface_albedo_updated =
                            (1 - metallic) * subsurface_albedo;  // since imetals do not do subsurface reflection
                    }

                    // Diffuse portion of reflection
                    reflected_color += (make_float3(1.f) - F) * subsurface_albedo_updated * incoming_light_ray;
                    float D = NormalDist(NdH, roughness);        // 1/pi omitted
                    float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
                    float3 f_ct = F * D * G;
                    reflected_color += f_ct * incoming_light_ray;
                }
            }
        }
    }

    // Correct Light reflected color contribution
    reflected_color = reflected_color * prd_camera->contrib_to_pixel * mat.transparency;

    //=================
    // Ambient light
    //=================
    // ambient light model is partial "flashlight" ambient light, partially from normal direction
    float3 ambient_light = params.ambient_light_color *
                           (make_float3(NdV) + make_float3(Dot(world_normal, make_float3(0, 0, 1)) * .5f + .5f)) *
                           subsurface_albedo * prd_camera->contrib_to_pixel * mat.transparency;

    //=================
    // If the surface is very smoooth, trace the reflected direction
    // Do this reflection regardless of GI on or off.
    //=================
    bool mirror_reflection = false;
    float3 mirror_reflection_color = make_float3(0.0);
    float3 next_dir = reflect(ray_dir, world_normal);
    next_dir = normalize(next_dir);

    float NdL = Dot(world_normal, next_dir);
    float3 halfway = normalize(next_dir - ray_dir);

    float NdH = Dot(world_normal, halfway);
    float VdH = Dot(-ray_dir, halfway);  // Same as LdH

    float3 F = make_float3(0.0f);
    float3 subsurface_albedo_updated = subsurface_albedo;
    // === dielectric workflow
    if (use_specular_workflow) {
        float3 F0 = specular * 0.08f;
        F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
    } else {
        float3 default_dielectrics_F0 = make_float3(0.04f);
        F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
        subsurface_albedo_updated = (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection
    }

    float D = NormalDist(NdH, roughness);        // 1/pi omitted
    float G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted

    float3 f_ct = F * D * G;

    // Note only specular part appears here. Energy preserve
    // Since it is not random, PDF is 1 (normally 1/pi),
    float3 next_contrib_to_pixel = f_ct * NdL;
    next_contrib_to_pixel = clamp(next_contrib_to_pixel / (4 * CUDART_PI_F), make_float3(0), make_float3(1));

    // If the camera uses GI, then it will trace two rays. So each ray's contribution should be halfed
    if (prd_camera->use_gi) {
        next_contrib_to_pixel = next_contrib_to_pixel * 0.5f;
    }
    // corrected for transparency
    next_contrib_to_pixel = next_contrib_to_pixel * mat.transparency * prd_camera->contrib_to_pixel;

    if (luminance(next_contrib_to_pixel) > params.importance_cutoff && prd_camera->depth + 1 < params.max_depth) {
        mirror_reflection = true;
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

        // mirror correction accounts for us oversampling this direction
        // following line comes from a heuristic. Perect reflection for metalic smooth objects,
        // no reflection for rough non-metalic objects
        float mirror_correction = (1.f - roughness) * (1.f - roughness) * metallic * metallic;
        mirror_reflection_color = prd_reflection.color * mirror_correction;
    }

    //=================
    // Global illumination ray.
    //=================
    float3 gi_reflection_color = make_float3(0);
    if (prd_camera->use_gi) {
        // sample hemisphere for next ray when using global illumination
        float z1 = curand_uniform(&prd_camera->rng);
        float z2 = curand_uniform(&prd_camera->rng);
        next_dir = sample_hemisphere_dir(z1, z2, world_normal);

        NdL = Dot(world_normal, next_dir);
        halfway = normalize(next_dir - ray_dir);

        NdH = Dot(world_normal, halfway);
        VdH = Dot(-ray_dir, halfway);  // Same as LdH

        F = make_float3(0.0f);
        subsurface_albedo_updated = subsurface_albedo;
        // === dielectric workflow
        if (use_specular_workflow) {
            float3 F0 = specular * 0.08f;
            F = fresnel_schlick(VdH, 5.f, F0, make_float3(1.f) /*make_float3(fresnel_max) it is usually 1*/);
        } else {
            float3 default_dielectrics_F0 = make_float3(0.04f);
            F = metallic * subsurface_albedo + (1 - metallic) * default_dielectrics_F0;
            subsurface_albedo_updated =
                (1 - metallic) * subsurface_albedo;  // since metals do not do subsurface reflection
        }

        D = NormalDist(NdH, roughness);        // 1/pi omitted
        G = HammonSmith(NdV, NdL, roughness);  // 4  * NdV * NdL omitted
        f_ct = F * D * G;
        // Specular part
        next_contrib_to_pixel = f_ct * NdL;

        // If mirror_reflection, then it will trace two rays. So each ray's contribution should be halfed
        if (mirror_reflection) {
            next_contrib_to_pixel = next_contrib_to_pixel * 0.5f;
        }
        // Diffuse part
        F = clamp(F, make_float3(0), make_float3(1));
        next_contrib_to_pixel += (make_float3(1.f) - F) * subsurface_albedo_updated * NdL;

        // Corrected for current transparency
        next_contrib_to_pixel = next_contrib_to_pixel * mat.transparency * prd_camera->contrib_to_pixel;

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
            gi_reflection_color = prd_reflection.color;  // accumulate indirect lighting color
        }
    }

    //=================
    // Combine all tracing light together
    //=================
    reflected_color = reflected_color + mirror_reflection_color;

    prd_camera->color = reflected_color + refracted_color;

    prd_camera->color += prd_camera->use_gi ? gi_reflection_color : ambient_light;

    if (prd_camera->depth == 2) {
        prd_camera->albedo = subsurface_albedo;
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
            RadarShader(getRadarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir,
                        mat_params->translational_velocity, mat_params->angular_velocity, mat_params->objectId);
            break;
        case SHADOW_RAY_TYPE:
            ShadowShader(getShadowPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
        case SEGMENTATION_RAY_TYPE:
            SemanticShader(getSemanticPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
            break;
    }
}
