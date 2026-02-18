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
// Device-side functions related to rectangle light in OptiX shaders.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_RECTANGLE_LIGHT_CU
#define CHRONO_SENSOR_OPTIX_RECTANGLE_LIGHT_CU

#include "chrono_sensor/optix/shaders/ChOptixLightStructs.h"	// for RectangleLightData, LightSample
#include "chrono_sensor/optix/ChOptixDefinitions.h"				// for PerRayData_camera, ContextParameters
#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"			// for SampleCosineHemisphereDir(...)

/// @brief Analytically check whether a ray intersects with a rectangle. If it does, return the hit distance `t_hit` and hit point `p_hit`.
/// @param ray_o origin of the ray
/// @param ray_d direction of the ray (should be normalized)
/// @param rect_center center position of the rectangle
/// @param length_vec vector of the length side of the rectangle
/// @param width_vec vector of the width side of the rectangle
/// @param rect_n normal vector of the rectangle (should be normalized), pointing toward the light-emitting direction
/// @param t_hit output hit distance along the ray (from ray_o to p_hit); valid only if the function returns true
/// @param p_hit output hit point on the rectangle; valid only if the function returns true
/// @return True if the ray intersects the rectangle, false otherwise
static __device__ __inline__ bool IntersectRectangle(
	const float3& ray_o, const float3& ray_d, const float3& rect_center,
	const float3& length_vec, const float3& width_vec, const float3& rect_n,
	float& t_hit, float3& p_hit
) {
	const float denom = Dot(ray_d, rect_n);
	if (fabsf(denom) < 1e-8f) {
		t_hit = 0.f;
		return false;
	}

	t_hit = Dot(rect_center - ray_o, rect_n) / denom;
	if (t_hit <= 0.f) {
		t_hit = 0.f;
		return false;
	}

	p_hit = ray_o + t_hit * ray_d;

	// Check bounds in parallelogram coordinates:
	// p = center + a * length_vec + b * width_vec, with a,b in [-0.5, 0.5]
	const float3 rel = p_hit - rect_center;
	const float a = Dot(rel, length_vec) / Dot(length_vec, length_vec);
	const float b = Dot(rel,  width_vec) / Dot( width_vec,  width_vec);

	if (a < -0.5f || a > 0.5f || b < -0.5f || b > 0.5f) {
		return false;
	}

	return true;
}

//// ---- main function ---- ////

/// @brief Check visibility between the rectangle light and the hit point, and sample the light.
/// @param cntxt_params context parameters
/// @param light_data rectangle light data
/// @param light_posi position of the rectangle light
/// @param light_sample the light sample to be updated
/// @return True if the light is visible from the hit point, false otherwise
static __device__ __inline__ bool CheckVisibleAndSampleRectangleLight(
	const ContextParameters& cntxt_params,
	const RectangleLightData& light_data,
	const float3& light_posi,
	LightSample& light_sample,
	PerRayData_camera* prd_camera
) {
	
	// Heuristic mixing probability p_light: based on approximate solid angle of the rectangle.
	// omega ~= light_data.area * cos(theta_light) / dist^2. Use a simple mapping omega / ( omega + 1).
	// We'll estimate using the center direction first.
	float3 dir_hp_to_light	= light_posi - light_sample.hitpoint;
	float  dist_hp_light	= Length(dir_hp_to_light);
	dir_hp_to_light			= dir_hp_to_light / dist_hp_light;
	float  cosL_center		= fmaxf(0.0f, Dot(light_data.light_dir, -dir_hp_to_light));
	float  omega_approx		= light_data.area * cosL_center / (dist_hp_light * dist_hp_light);
	float  P_light			= omega_approx / (omega_approx + 1.f); // magic formula
	       P_light			= clamp(P_light, 0.10f, 0.90f);  // keep stable

	// Randomly choose a sampling strategy based on p_light
	const bool choose_light = (curand_uniform(&prd_camera->rng) < P_light);

	// We'll compute a candidate (dir, dist, NdL, cos_on_light, pdf_light_omega, pdf_dir)
	float  cos_on_light = 0.f;
	float  pdf_light_omega = 0.f;
	float  pdf_hp_dir = 0.f;
	float3 p_on_light = make_float3(0.f, 0.f, 0.f);

	// --- Strategy 1: Uniform point on rectangle area ---
	if (choose_light) {
		
		// Sample in [-0.5, 0.5]^2
		const float su = curand_uniform(&prd_camera->rng) - 0.5f;
		const float sv = curand_uniform(&prd_camera->rng) - 0.5f;
		p_on_light = light_posi + su * light_data.length_vec + sv * light_data.width_vec;

		// Direction and distance from hit-point to light point
		light_sample.dir = p_on_light - light_sample.hitpoint;
		light_sample.dist = Length(light_sample.dir);
		light_sample.dir = light_sample.dir / light_sample.dist;
		light_sample.NdL = Dot(light_sample.n, light_sample.dir);

		cos_on_light = Dot(light_data.light_dir, -light_sample.dir);
		// Check if light point is below the surface (surface cosine), or hit-point is behind the rectangle light (emitter cosine)
		if (light_sample.NdL < 0.f || cos_on_light < 0.f) {
			light_sample.L = make_float3(0.f, 0.f, 0.f);
			light_sample.pdf = 0.f;
			return false;
		}

		// PDF for uniform area -> solid angle
		// pdf_A = 1/area  ---> pdf_omega = dist^2 / (cos_on_light * area)
		pdf_light_omega = (light_sample.dist * light_sample.dist) / (cos_on_light * light_data.area);

		// PDF for cosine hemisphere (same direction) of the hit-point, used in mixture PDF. 
		// pdf_hp_dir = cos(theta) / pi, where theta is wrt surface normal
		pdf_hp_dir = light_sample.NdL / CUDART_PI_F;

	}
	// --- Strategy 2: cosine-weighted hemisphere direction from hit point ---
	else {

		light_sample.dir = SampleCosineHemisphereDir(curand_uniform(&prd_camera->rng), curand_uniform(&prd_camera->rng), light_sample.n);
		light_sample.NdL = Dot(light_sample.n, light_sample.dir);

		cos_on_light = Dot(light_data.light_dir, -light_sample.dir);
		// Check if hit-point is behind the rectangle light (emitter cosine)
		if (cos_on_light < 0.f) {
			light_sample.L = make_float3(0.f, 0.f, 0.f);
			light_sample.pdf = 0.f;
			return false;
		}

		// Intersect this direction with the rectangle analytically
		if (!IntersectRectangle(
			light_sample.hitpoint, light_sample.dir, light_posi, light_data.length_vec, light_data.width_vec, light_data.light_dir, light_sample.dist, p_on_light
		)) {
			// Direction sample does not hit the rectangle; no contribution.
			light_sample.L = make_float3(0.f, 0.f, 0.f);
			light_sample.pdf = 0.f;
			return false;
		}

		// PDF of this sampled direction
		pdf_hp_dir = light_sample.NdL / CUDART_PI_F;

		// pdf_light_omega for this same direction (if it had been generated by area sampling)
		pdf_light_omega = (light_sample.dist * light_sample.dist) / (cos_on_light * light_data.area);
	}

	// Trace occlusion ray toward the sampled point on the rectangle (distance-limited)
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
		light_sample.dist,			// A very large max distance (effectively “infinite” for the scene scale)
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
	// Evaluate rectangle emitter contribution
	else {
		// Use Lambertian emitter geometry term: (cos_on_light / dist^2)
		float geom_term = cos_on_light * ((light_data.const_color) ? 1.f : (light_data.atten_scale / (light_sample.dist * light_sample.dist)));

		// Write outputs
		light_sample.L = light_sample.NdL * geom_term * light_data.color;
		
		// Mixture PDF in solid angle measure
		light_sample.pdf = P_light * pdf_light_omega + (1.0f - P_light) * pdf_hp_dir;

		return true;
	}
}

/// @brief Check and visualize the rectangle light of a first-hit ray.
/// @param cntxt_params context parameters
/// @param ray_o origin of the ray
/// @param ray_d direction of the ray (should be normalized)
/// @param light_data rectangle light data
/// @param light_posi position of the rectangle light center
/// @param t_hit returned hit distance along the ray (from ray_o to p_hit); valid only if the function returns true
/// @param color returned visualized color of the rectangle light; valid only if the function returns true
/// @return True if the ray intersects the rectangle light, false otherwise
static __device__ __inline__ bool CheckVisualizeRectangleLight(			
	const ContextParameters& cntxt_params,
	const float3& ray_o,
	const float3& ray_d, 
	const RectangleLightData& light_data,
	const float3& light_posi,
	float& t_hit,
	float3& color
) {	
	float3 p_hit = make_float3(0.f, 0.f, 0.f);
	// Examine if the ray intersects the rectangle light
	if(IntersectRectangle(
		ray_o, ray_d, light_posi, light_data.length_vec, light_data.width_vec, light_data.light_dir, t_hit, p_hit
	)) {

		float cos_on_light = Dot(light_data.light_dir, -ray_d);
		
		// The ray hits the back face of the rectangle light
		if (cos_on_light < 0.f) {
			color = make_float3(0.f, 0.f, 0.f);
		}
		// The ray hits the front face of the rectangle light
		else {
			float geom_term = cos_on_light * ((light_data.const_color) ? 1.f : (light_data.atten_scale / (t_hit * t_hit)));
			color = geom_term * light_data.color;
		}
		
		return true;
		
	}
	// Ray does not intersect the rectangle light at all
	else {
		t_hit = 0.f;
		color = make_float3(0.f, 0.f, 0.f);
		return false;
	}
}

#endif  // CHRONO_SENSOR_OPTIX_RECTANGLE_LIGHT_CU