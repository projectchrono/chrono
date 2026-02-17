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
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Camera shader
//
// =============================================================================

#ifndef CAMERA_SHADER_CU
#define CAMERA_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"
#include "chrono_sensor/optix/shaders/camera_hapke_shader.cu"
#include "chrono_sensor/optix/shaders/camera_volumetric_shader.cu"
#include "chrono_sensor/optix/shaders/camera_legacy_shader.cu"
#include "chrono_sensor/optix/shaders/camera_path_shader.cu"


/// @brief Get the PerRayData of the physics-based camera
/// @return Pointer to the PerRayData_phys_camera struct
__device__ __inline__ PerRayData_phys_camera* GetPhysCameraPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_phys_camera*>(ints_as_pointer(opt0, opt1));
}

/// @brief Get the PerRayData of the camera
/// @return Pointer to the PerRayData_camera struct
__device__ __inline__ PerRayData_camera* GetCameraPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_camera*>(ints_as_pointer(opt0, opt1));
}

/// @brief Camera shader that handles different BSDF types and integrators
/// @param cntxt_params Context parameters containing scene information
/// @param prd_camera Pointer to the PerRayData_camera struct
/// @param mat_params Material parameters of the hit object
/// @param material_id ID of the material of the hit object
/// @param world_normal Normal vector in world coordinates of the hit point
/// @param uv UV coordinates of the hit point on the surface of the object
/// @param tangent Tangent vector in world coordinates at the hit point
/// @param ray_dist Distance from the ray origin to the hit point
/// @param hit_point Position in world coordinates of the hit point
/// @param ray_dir Direction vector in world coordinates of the incoming ray
/// @param ray_origin Position in world coordinates of the ray origin
static __device__ __inline__ void CameraShader(
	const ContextParameters& cntxt_params, PerRayData_camera* prd_camera, const MaterialParameters& mat_params, 
	const MaterialRecordParameters* mat_record_params, unsigned int& material_id, const float3& world_normal, const float2& uv,
	const float3& tangent, const float& ray_dist, const float3& hit_point, const float3& ray_dir, const float3& ray_origin
) {
	// if (prd_cam->depth > 2) { // debug
	//     printf("CH |d: %d t: %f | o: (%f,%f,%f) | d: (%f,%f,%f) | hp: (%f,%f,%f) | n: (%f,%f,%f)\n",
	//            prd_cam->depth, ray_dist, ray_orig.x, ray_orig.y, ray_orig.z, ray_dir.x, ray_dir.y, ray_dir.z, hp.x, hp.y,
	//            hp.z, world_normal.x, world_normal.y, world_normal.z);
	// }

	switch (mat_params.bsdf_type) {
		case BSDFType::HAPKE:
		case BSDFType::VDBHAPKE: {
			CameraHapkeShader(
				prd_camera, mat_record_params, material_id, mat_record_params->num_blended_materials, world_normal, uv, tangent, ray_dist,
				hit_point, ray_dir
			);
			break;
		}

		case BSDFType::VDBVOL: {
			CameraVolumetricShader(
				prd_camera, mat_record_params, material_id, mat_record_params->num_blended_materials, world_normal, uv, tangent, ray_dist,
				ray_origin, ray_dir);
			break;
		}

		default: {
			switch (prd_camera->integrator) {
				case Integrator::PATH: {
					CameraPathIntegrator(
						cntxt_params, prd_camera, mat_record_params, material_id, world_normal, uv, tangent, ray_dist, hit_point, ray_dir,
						ray_origin
					);
					break;
				}

				case Integrator::LEGACY: {
					CameraLegacyShader(
						cntxt_params, prd_camera, mat_record_params, material_id, mat_record_params->num_blended_materials, world_normal, uv,
						tangent, ray_dist, hit_point, ray_dir
					);
					break;
				}

				default:
					break;
			}
			break;
		}
	}
}

# endif  // CAMERA_SHADER_CU