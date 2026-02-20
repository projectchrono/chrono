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
// Device-side light class base for different types of lights in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_LIGHT_HUBS_CU
#define CHRONO_SENSOR_OPTIX_LIGHT_HUBS_CU

#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"
#include "chrono_sensor/optix/shaders/ChOptixPointLight.cu"
#include "chrono_sensor/optix/shaders/ChOptixDirectionalLight.cu"
#include "chrono_sensor/optix/shaders/ChOptixSpotLight.cu"
#include "chrono_sensor/optix/shaders/ChOptixRectangleLight.cu"
#include "chrono_sensor/optix/shaders/ChOptixDiskLight.cu"
#include "chrono_sensor/optix/shaders/ChOptixEnvironmentLight.cu"

//// ---- Register Your Customized Light Type Here (include the corresponding .cu file) ---- ////

// Check if the light source is visible to the hit-point
static __device__ __inline__ bool CheckVisibleAndSampleLight(
	const ContextParameters& cntxt_params, const ChOptixLight& light, LightSample& light_sample,
	PerRayData_camera* prd_camera
) {
	switch (light.light_type) {
        case LightType::POINT_LIGHT: {
            bool flag = CheckVisibleAndSamplePointLight(
				cntxt_params, light.specific.point, light.pos, light_sample
				// prd_camera // debug
			);
			// prd_camera->color = {flag * 1.0, 0.0, 1.0}; // debug
			return flag;
            break;
		}

        case LightType::SPOT_LIGHT: {
            bool flag = CheckVisibleAndSampleSpotLight(
				cntxt_params, light.specific.spot, light.pos, light_sample
				// prd_camera // debug
			);
			// prd_camera->color = {flag * 1.0, 0.0, 1.0}; // debug
			return flag;
			break;
		}

		case LightType::DIRECTIONAL_LIGHT: {
            bool flag = CheckVisibleAndSampleDirectionalLight(
				cntxt_params, light.specific.directional, light_sample
				// prd_camera // debug
			);
			// prd_camera->color = {flag * 1.0, 0.0, 1.0}; // debug
			return flag;
			break;
		}
		
		case LightType::RECTANGLE_LIGHT: {
            bool flag = CheckVisibleAndSampleRectangleLight(
				cntxt_params, light.specific.rectangle, light.pos, light_sample, prd_camera
			);
			// prd_camera->color = {flag * 1.0, 0.0, 1.0}; // debug
			return flag;
			break;
		}
		
		case LightType::DISK_LIGHT: {
            bool flag = CheckVisibleAndSampleDiskLight(
				cntxt_params, light.specific.disk, light.pos, light_sample, prd_camera
			);
			// prd_camera->color = {flag * 1.0, 0.0, 1.0}; // debug
			return flag;
			break;
		}
		
		case LightType::ENVIRONMENT_LIGHT: {
            bool flag = CheckVisibleAndSampleEnvironmentLight(
				cntxt_params, light.specific.environment, light_sample, prd_camera
			);
			// prd_camera->color = {flag * 1.0, 0.0, 1.0}; // debug
			return flag;
			break;
		}

		//// ---- Register Your Customized Light Type Here (check visibility and sample light) ---- ////
		
		default: {
			printf("Light type not recognized in visibility check ...... \n");
			return false;
            break;
		}
    }
}

static __device__ __inline__ bool CheckVisualizeNonDeltaLight(
	const ContextParameters& cntxt_params, const float3& ray_o, const float3& ray_d, const ChOptixLight& light,
	float& t_hit, float3& color, float3& light_albedo, float3& light_normal
) {
	switch (light.light_type) {
        case LightType::RECTANGLE_LIGHT: {
			light_albedo = light.specific.rectangle.color;
			light_normal = light.specific.rectangle.light_dir;
			return CheckVisualizeRectangleLight(cntxt_params, ray_o, ray_d, light.specific.rectangle, light.pos, t_hit, color);
			break;
		}

		case LightType::DISK_LIGHT: {
			light_albedo = light.specific.disk.color;
			light_normal = light.specific.disk.light_dir;
			return CheckVisualizeDiskLight(cntxt_params, ray_o, ray_d, light.specific.disk, light.pos, t_hit, color);
			break;
		}

		case LightType::ENVIRONMENT_LIGHT: {
			return false;
			break;
		}

		default: {
			printf("Light type not recognized in non-delta light visualization ...... \n");
			return false;
			break;
		}
	}
}
# endif // CHRONO_SENSOR_OPTIX_LIGHT_HUBS_CU