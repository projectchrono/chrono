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
// Device-side functions related to directional light in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_DIRECTIONAL_LIGHT_CU
#define CHRONO_SENSOR_OPTIX_DIRECTIONAL_LIGHT_CU

#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h" // for DirectionalLightData, LightSample
#include "chrono_sensor/optix/ChOptixDefinitions.h" // for PerRayData_camera, ContextParameters
#include "chrono_sensor/optix/shaders/device_utils.h"


/// @brief Check visibility between the hit point along the directional light direction, and sample the light.
/// @param cntxt_params context parameters
/// @param light_data directional light data
/// @param light_sample the light sample to be updated
/// @return True if the light is visible from the hit point, false otherwise
static __device__ __inline__ bool CheckVisibleAndSampleDirectionalLight(
	const ContextParameters& cntxt_params,
	const DirectionalLightData& light_data,
	LightSample& light_sample
	// const PerRayData_camera* prd_camera // debug
) {
	// Direction and distance from hit-point to light
	light_sample.dir = light_data.light_dir;
	light_sample.NdL = Dot(light_sample.n, light_sample.dir);
	
	// Light is below the surface
	if (light_sample.NdL < 0) {
		light_sample.L = make_float3(0.f, 0.f, 0.f);
		light_sample.pdf = 0.f;
		return false;  
	}

	// Trace shadow ray toward the light to check for occlusion		
	PerRayData_occlusion prd_occ;
	prd_occ.occluded = false;

	unsigned int opt1;
	unsigned int opt2;
	pointer_as_ints(&prd_occ, opt1, opt2);

	// Payload 2: ray type (if your code uses it)
	unsigned int raytype = (unsigned int)RayType::OCCLUSION_RAY_TYPE;

	optixTrace(
		cntxt_params.root,          // The scene traversable handle (OptixTraversableHandle); basically the top-level acceleration structure (TLAS).
		light_sample.hitpoint, 		// origin of the traced ray
		light_sample.dir,          	// direction of the traced ray
		cntxt_params.scene_epsilon, // minimum intersection distance to avoid self-intersection (“shadow acne”)
		1e16f,						// A very large max distance (effectively “infinite” for the scene scale)
		optixGetRayTime(),          // time value for launching this ray
		OptixVisibilityMask(1),     // Only intersects geometry whose instance mask matches 1
		OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, // terminate on first hit is ideal for occlusion rays
		0,                          // SBT offset (used when you have multiple SBT records for the same ray type). It selects the first “ray type slot”
		1,                          // SBT stride (used when you have multiple SBT records for the same ray type). It usually means “one ray type stride”
		0,                          // missSBTIndex. It selects the first miss program
		opt1,                       // Final payloads; the per-ray data pointer (first 32 bits); optixGetPayload_0() = opt1
		opt2,                       // Final payloads; the per-ray data pointer (second 32 bits); optixGetPayload_1() = opt2
		raytype                     // The ray type index (used when you have multiple ray types, e.g., radiance rays, shadow rays, etc.)
	);
	
	// If the light is occluded
	if (prd_occ.occluded) {
		light_sample.L = make_float3(0.f, 0.f, 0.f);
		light_sample.pdf = 0.f;
		return false;
	}
	// Caculate the remaining attributes of light sample
	else {
		light_sample.L = light_sample.NdL * light_data.color;
		light_sample.pdf = 1.0f; // Delta light
		return true;
	}
}

#endif  // CHRONO_SENSOR_OPTIX_DIRECTIONAL_LIGHT_CU