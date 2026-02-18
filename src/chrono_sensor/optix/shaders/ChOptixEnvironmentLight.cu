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
// Device-side functions related to environment light in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_ENVIRONMENT_LIGHT_CU
#define CHRONO_SENSOR_OPTIX_ENVIRONMENT_LIGHT_CU

#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h" // for EnvironmentLightData, LightSample
#include "chrono_sensor/optix/ChOptixDefinitions.h" // for PerRayData_camera, ContextParameters
#include "chrono_sensor/optix/shaders/device_utils.h"


/// @brief Binary search: returns smallest idx in [`bias`, `bias` + `n` - 1] such that cdf[idx] >= threshold.
/// Assumes cdf is non-decreasing, and cdf[n-1] == 65535.
/// @param cdf input CDF array (quantized to [0, 65535])
/// @param bias starting index of the search range in `cdf`
/// @param n number of elements in the search range
/// @param threshold random threshold in [0, 65535] for sampling
/// @return the sampled index in [`bias`, `bias` + `n` - 1]
static __device__ __inline__ int SampleCDF_u16(
    const unsigned short* cdf, const int& bias, const int n, unsigned short threshold
) {
    int idx_low = bias;
    int idx_high = bias + n - 1;
    while (idx_low < idx_high) {
        int idx_mid = (idx_low + idx_high) >> 1;
        // Note: cdf[] values are in [0, 65535]
        if ((unsigned short)cdf[idx_mid] < threshold) {
            idx_low = idx_mid + 1;
        }
        else {
            idx_high = idx_mid;
        }
    }
    return idx_low;
}

/// @brief Get probability mass of bin idx for a quantized CDF in [0, 65535].
/// @param cdf input CDF array (quantized to [0, 65535])
/// @param idx the index of the bin for which to compute the probability mass
/// @return probability mass of bin idx.
static __device__ __inline__ float CDFDelta_u16(const unsigned short* cdf, const int& idx) {
    
    unsigned short prev = (idx == 0) ? 0 : cdf[idx - 1];
    return (float)(cdf[idx] - prev) / 65535.f;
}

/// @brief Sample an environment direction using two-stage importance sampling.
/// @param light_data environment light data containing the CDFs and environment map
/// @param prd_camera per-ray data for the camera ray, in which random number generator is used
/// @param dir_out output sampled direction (will be normalized)
/// @param pdf_omega_out output PDF value in solid angle measure for the sampled direction
static __device__ __inline__ void SampleEnvironmentDirection(
    const EnvironmentLightData& light_data,
    PerRayData_camera* prd_camera,
    float3& dir_out,
    float& pdf_omega_out,
    float2& tex_xy_out
) {
    const int img_w = light_data.width;
    const int img_h = light_data.height;

	    // --- 1) Sample latitude v using marginal CDF ---
    float z0 = static_cast<unsigned short>(curand_uniform(&prd_camera->rng) * 65535.f); // (0, 65535]
    int v = SampleCDF_u16(light_data.dev_cdf_lat, 0, img_h, z0);
    float prob_v = CDFDelta_u16(light_data.dev_cdf_lat, v);

    // --- 2) Sample longitude column u using conditional CDF for row v ---
    unsigned short z1 = static_cast<unsigned short>(curand_uniform(&prd_camera->rng) * 65535.f); // (0, 65535]
    int u = SampleCDF_u16(light_data.dev_cdf_lon, v * img_w, img_w, z1);
    float prob_u_given_v = CDFDelta_u16(light_data.dev_cdf_lon, u);
    u = u - v * img_w; // convert to [0, img_w - 1]

    // Jitter within the pixel for continuous sampling in UV domain
    float ju = curand_uniform(&prd_camera->rng);  // (0, 1]
    float jv = curand_uniform(&prd_camera->rng);  // (0, 1]

    // IMPORTANT: to match miss.cu's tex_x range [-0.5, 0.5],
    // we generate tex_x directly in that range.
    tex_xy_out = make_float2((u + ju) / (float)img_w, (v + jv) / (float)img_h); // in (0, 1] x (0, 1]

    // Convert UV tex coords to direction:
    float azimuth = 2.0f * CUDART_PI_F * (tex_xy_out.x - 0.5f); // in [-pi, pi]
    float elevation =      CUDART_PI_F * (tex_xy_out.y - 0.5f); // in [-pi/2, pi/2]

    dir_out = make_float3(cosf(azimuth) * cosf(elevation), sinf(azimuth) * cosf(elevation), sinf(elevation));

	// prd_camera->color = make_float3(0.f, 0.f, 1.f); // debug
	// return; // debug

    // --- PDF computation ---
    pdf_omega_out = prob_v * prob_u_given_v * (img_w * img_h);
}


/// @brief Check visibility between the hit point along the environment light direction, and sample the light.
/// @param cntxt_params context parameters
/// @param light_data environment light data
/// @param light_sample the light sample to be updated
/// @return True if the light is visible from the hit point, false otherwise
static __device__ __inline__ bool CheckVisibleAndSampleEnvironmentLight(
	const ContextParameters& cntxt_params,
    const EnvironmentLightData& light_data,
    LightSample& light_sample,
    PerRayData_camera* prd_camera
) {

	// if (/*!light_data.env_map ||*/ !light_data.dev_cdf_lon) {
	// 	light_sample.pdf = 1.f;
	// 	light_sample.L = make_float3(0.f, 0.f, 1.f);
	// 	return true;
	// }


    // Sample direction and probabilidy density function (PDF) from the environment map using two-stage importance sampling
    float2 tex_xy = make_float2(0.f, 0.f); // in (0, 1] x (0, 1]
    SampleEnvironmentDirection(light_data, prd_camera, light_sample.dir, light_sample.pdf, tex_xy);

    // Update geometric terms
    light_sample.dist = 1e16f;  // "infinite" distance
    light_sample.NdL = Dot(light_sample.n, light_sample.dir);

    // Light is below the surface
    if (light_sample.NdL <= 0.f || light_sample.pdf <= 0.f) {
        light_sample.L = make_float3(0.f, 0.f, 0.f);
        light_sample.pdf = 0.f;
        return false;
    }

    // Trace shadow ray toward the sampled direction to check for occlusion
    PerRayData_occlusion prd_occ;
    prd_occ.occluded = false;

    unsigned int opt1, opt2;
    pointer_as_ints(&prd_occ, opt1, opt2);

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

    // If occluded, no contribution
    if (prd_occ.occluded) {
        light_sample.L = make_float3(0.f, 0.f, 0.f);
        light_sample.pdf = 0.f;
        return false;
    }

    // Evaluate environment radiance along sampled direction
    float4 rgba = tex2D<float4>(light_data.env_map, tex_xy.x, tex_xy.y);
    
    // Convert texture value to linear like miss.cu and scale
    light_sample.L = make_float3(rgba.x, rgba.y, rgba.z) * light_data.intensity_scale * light_sample.NdL;

    return true;
}
	
/*	
	const ContextParameters& cntxt_params,
	const EnvironmentLightData& light_data,
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
*/

#endif  // CHRONO_SENSOR_OPTIX_ENVIRONMENT_LIGHT_CU