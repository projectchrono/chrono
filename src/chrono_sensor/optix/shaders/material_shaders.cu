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

#include "chrono_sensor/optix/shaders/device_utils.cuh"
#include "chrono_sensor/optix/shaders/shader_utils.cuh"

#include "chrono_sensor/optix/shaders/shadow_shader.cuh"
#include "chrono_sensor/optix/shaders/normal_cam_shader.cuh"
#include "chrono_sensor/optix/shaders/depth_cam_shader.cuh"
#include "chrono_sensor/optix/shaders/segment_cam_shader.cuh"
#include "chrono_sensor/optix/shaders/radar_shader.cuh"
#include "chrono_sensor/optix/shaders/lidar_shader.cuh"
#include "chrono_sensor/optix/shaders/camera_shader.cuh"


#ifdef USE_SENSOR_NVDB
    #include <nanovdb/NanoVDB.h>
    #include <nanovdb/util/Ray.h>
    #include <nanovdb/util/HDDA.h>
#endif

extern "C" __global__ void __closesthit__material_shader() {
    // printf("Material Shader!\n");
    //  determine parameters that are shared across all ray types
    const MaterialRecordParameters* mat_record_params = (MaterialRecordParameters*)optixGetSbtDataPointer();

    const float3 ray_orig = optixGetWorldRayOrigin();
    const float3 ray_dir = normalize(optixGetWorldRayDirection());  // this may be modified by the scaling transform
    const float ray_dist = optixGetRayTmax();
    const float3 hit_point = ray_orig + ray_dir * ray_dist;

    // printf("NVDBVolShader: orig: (%f,%f,%f), dir:(%f,%f,%f)\n", ray_orig.x, ray_orig.y, ray_orig.z, ray_dir.x, ray_dir.y, ray_dir.z);
    float3 object_normal;
    float2 uv;
    float3 tangent;

    // check if we hit a triangle
    unsigned int material_id = mat_record_params->material_pool_id;
    const MaterialParameters& mat = params.material_pool[material_id];
    if (optixIsTriangleHit()) {
        GetTriangleData(object_normal, material_id, uv, tangent, mat_record_params->mesh_pool_id);
    }
    else {
        object_normal = make_float3(__int_as_float(optixGetAttribute_0()), __int_as_float(optixGetAttribute_1()),
                                    __int_as_float(optixGetAttribute_2()));
        uv = make_float2(__int_as_float(optixGetAttribute_3()), __int_as_float(optixGetAttribute_4()));
        tangent = make_float3(__int_as_float(optixGetAttribute_5()), __int_as_float(optixGetAttribute_6()),
                              __int_as_float(optixGetAttribute_7()));
    }

    if (mat.kn_tex) {
        float3 bitangent = normalize(Cross(object_normal, tangent));
        const float4 tex = tex2D<float4>(mat.kn_tex, uv.x * mat.tex_scale.x, uv.y * mat.tex_scale.y);
        float3 normal_delta = make_float3(tex.x, tex.y, tex.z) * 2.f - make_float3(1.f);
        object_normal =
            normalize(normal_delta.x * tangent + normal_delta.y * bitangent + normal_delta.z * object_normal);
    }

    float3 world_normal;
    if (mat.bsdf_type == BSDFType::VDB || mat.bsdf_type == BSDFType::VDBHAPKE) {
        world_normal = object_normal;
    }
    else {
        world_normal = normalize(optixTransformNormalFromObjectToWorldSpace(object_normal));
    }

    // float3 hp = ray_orig + ray_dist * ray_dir; // debug

    // From here on out, things are specific to the ray type
    RayType raytype = (RayType)optixGetPayload_2();
    if (Dot(world_normal, world_normal) > 0.f) {
        switch (raytype) {
            case RayType::OCCLUSION_RAY_TYPE: {
                PerRayData_occlusion* prd_occlusion = GetOcclusionPRD();
                prd_occlusion->occluded = true;
                break;
            }
            
            case RayType::CAMERA_RAY_TYPE: {
                CameraShader(params, GetCameraPRD(), mat, mat_record_params, material_id, world_normal, uv, tangent, ray_dist, hit_point, ray_dir, ray_orig);
                break;
            }

            case RayType::PHYS_CAMERA_RAY_TYPE: {
                PerRayData_phys_camera* prd_phys_camera_ptr = GetPhysCameraPRD();
                prd_phys_camera_ptr->distance = ray_dist;
                CameraShader(params, GetCameraPRD(), mat, mat_record_params, material_id, world_normal, uv, tangent, ray_dist, hit_point, ray_dir, ray_orig);
                break;
            }

            case RayType::LIDAR_RAY_TYPE: {
                LidarShader(GetLidarPRD(), mat, world_normal, ray_dist, ray_dir);
                break;
            }

            case RayType::RADAR_RAY_TYPE: {
                RadarShader(GetRadarPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir,
                            mat_record_params->translational_velocity, mat_record_params->angular_velocity, mat_record_params->objectId);
                break;
            }

            case RayType::SHADOW_RAY_TYPE: {
                ShadowShader(GetShadowPRD(), mat, world_normal, uv, tangent, ray_dist, ray_orig, ray_dir);
                break;
            }

            case RayType::SEGMENTATION_RAY_TYPE: {
                SegmentCamShader(GetSegmentPRD(), mat);
                break;
            }
            
            case RayType::DEPTH_RAY_TYPE: {
                DepthCamShader(GetDepthCameraPRD(), ray_dist);
                break;
            }

            case RayType::NORMAL_RAY_TYPE: {
                NormalCamShader(GetNormalCameraPRD(), world_normal);
                break;
            }

            default: {
                printf("Unsupported ray type in material shader ...... \n");
                break;
            }
                
            //// ---- Register Your Customized Sensor Here (case for your customized ray type) ---- ////
        }
    }
}
