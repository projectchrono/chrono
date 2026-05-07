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
// RT kernels for coloring upon ray not intersecting anything
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.cuh"
#include "chrono_sensor/optix/shaders/shader_utils.cuh"
#include "chrono_sensor/optix/shaders/shadow_shader.cuh"
#include "chrono_sensor/optix/shaders/normal_cam_shader.cuh"
#include "chrono_sensor/optix/shaders/depth_cam_shader.cuh"
#include "chrono_sensor/optix/shaders/segment_cam_shader.cuh"
#include "chrono_sensor/optix/shaders/radar_shader.cuh"
#include "chrono_sensor/optix/shaders/lidar_shader.cuh"
#include "chrono_sensor/optix/shaders/camera_shader.cuh"

extern "C" __global__ void __miss__shader() {
    const MissParameters* miss = (MissParameters*)optixGetSbtDataPointer();

    // figure out type and determine miss parameter for this ray type
    RayType raytype = (RayType)optixGetPayload_2();

    switch (raytype) {
        case RayType::OCCLUSION_RAY_TYPE: {
            PerRayData_occlusion* prd = GetOcclusionPRD();
            prd->occluded = false;
            break;
        }
        
        case RayType::PHYS_CAMERA_RAY_TYPE: 
        case RayType::CAMERA_RAY_TYPE: {
            const CameraMissParameters& camera_miss = miss->camera_miss;
            PerRayData_camera* prd;
            if (raytype == RayType::PHYS_CAMERA_RAY_TYPE) {
                prd = reinterpret_cast<PerRayData_phys_camera*>(GetPhysCameraPRD());
            }
            else {
                prd = GetCameraPRD();
            }

            if (camera_miss.mode == BackgroundMode::ENVIRONMENT_MAP && camera_miss.env_map) {
                // evironment map assumes z up
                float3 ray_dir = optixGetWorldRayDirection();
                float azimuth = atan2f(ray_dir.y, ray_dir.x); // in [-pi, pi]
                float elevation = asinf(ray_dir.z); // in [-pi/2, pi/2]
                float tex_x =   azimuth / (2 * CUDART_PI_F) + 0.5f; // in [0, 1]
                float tex_y = elevation /      CUDART_PI_F  + 0.5f; // in [0, 1]
                float4 tex = tex2D<float4>(camera_miss.env_map, tex_x, tex_y);
                // Gamma Correction
                prd->color = Pow(make_float3(tex.x, tex.y, tex.z), 2.2) * prd->contrib_to_pixel;
            } else if (camera_miss.mode == BackgroundMode::GRADIENT) {  // gradient
                // gradient assumes z=up
                float3 ray_dir = optixGetWorldRayDirection();
                float mix = max(0.f, ray_dir.z);
                prd->color =
                    (mix * camera_miss.color_zenith + (1 - mix) * camera_miss.color_horizon) * prd->contrib_to_pixel;
            } else {  // default to solid color
                prd->color = camera_miss.color_zenith * prd->contrib_to_pixel;
            }

            // apply fog model
            if (prd->use_fog && params.fog_scattering > 0.f) {
                float blend_alpha = expf(-params.fog_scattering * optixGetRayTmax());
                prd->color = blend_alpha * prd->color + (1 - blend_alpha) * params.fog_color * prd->contrib_to_pixel;
            }

            break;
        }

        case RayType::LIDAR_RAY_TYPE: {
            // leave as default values
            break;
        }

        case RayType::RADAR_RAY_TYPE: {
            // leave as default values
            break;
        }

        case RayType::DEPTH_RAY_TYPE: {
            PerRayData_depthCamera* prd = GetDepthCameraPRD();
            prd->depth = prd->max_depth; // set miss hit to max depth
            break;
        }

        case RayType::NORMAL_RAY_TYPE: {
            // leave as default values
            break;
        }
    }
}