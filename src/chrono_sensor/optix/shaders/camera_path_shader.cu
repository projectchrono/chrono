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
// Path camera shader, with assuming
// 1. No transparent materials considered
// 2. NO blended materials considered
// 3. Only reflection considered, no refraction considered
// 4. No emissive materials considered
//
// =============================================================================

#ifndef CAMERA_PATH_SHADER_CU
#define CAMERA_PATH_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"
#include "chrono_sensor/optix/shaders/ChOptixLightHubs.cu"
#include "chrono_sensor/optix/shaders/camera_raygen.cu"

static __device__ __inline__ void RussianRoulette(curandState_t& rng, float3& contrib_to_pixel) {
	float p = fmaxf(0.05, fminf(fmaxf(contrib_to_pixel), 0.95));
	contrib_to_pixel = (curand_uniform(&rng) > p) ? make_float3(0.f) : (contrib_to_pixel / p);
}

	
/// @brief Camera path integrator
/// @param cntxt_params Context parameters containing scene information
/// @param prd_camera Pointer to the PerRayData_camera struct
/// @param mat_params Pointer to the MaterialRecordParameters struct containing material properties of the hit object
/// @param material_id Material ID of the hit object
/// @param world_normal Normal vector in world coordinates of the hit point
/// @param uv UV coordinates of the hit point on the surface of the object
/// @param tangent Tangent vector in world coordinates of the hit point
/// @param ray_dist Distance from ray origin to hit point
/// @param hit_point World coordinates of the hit point
/// @param ray_dir Direction of the incoming ray
/// @param ray_origin Origin of the incoming ray
static __device__ __inline__ void CameraPathIntegrator(
	const ContextParameters& cntxt_params, PerRayData_camera* prd_camera, const MaterialRecordParameters* mat_params,
	unsigned int& material_id, const float3& world_normal, const float2& uv, const float3& tangent,
	const float& ray_dist, const float3& hit_point, const float3& ray_dir, const float3& ray_origin
) {
	// if (prd_camera->depth >= 3)
	//     printf("PI| d: %d | contr: (%f,%f,%f)\n", prd_camera->depth, prd_camera->contrib_to_pixel.x,
	//            prd_camera->contrib_to_pixel.y, prd_camera->contrib_to_pixel.z);
	
	// Visualize non-delta light
	if (prd_camera->depth == 2) {
		bool hit_non_delta_light = false;
		float min_t_hit = 1e16f;
		float3 viz_light_color = make_float3(0.f);
		float3 light_albedo = make_float3(0.f);
		float3 light_normal = make_float3(0.f);

		for (unsigned int light_idx = 0; light_idx < cntxt_params.num_lights; light_idx++) {
			ChOptixLight light = cntxt_params.lights[light_idx];
			if (light.delta == false) {
				float t_hit = 1e17f;
				float3 color = make_float3(0.f);
				if (CheckVisualizeNonDeltaLight(cntxt_params, ray_origin, ray_dir, light, t_hit, color, light_albedo, light_normal)) {
					hit_non_delta_light = true;
					if (t_hit < min_t_hit) {
						min_t_hit = t_hit;
						viz_light_color = color;
					}
				}
			}
		}

		if (hit_non_delta_light && min_t_hit < ray_dist) {
			prd_camera->color = viz_light_color;
			prd_camera->albedo = light_albedo;
			prd_camera->normal = light_normal;
			return;
		}
	}


	// Get material parameters
	const MaterialParameters& mat = params.material_pool[material_id];
	BSDFType bsdf_type = mat.bsdf_type;
	float3 albedo	= (mat.kd_tex) 			? GetTexFloat4ValFloat3(mat.kd_tex, mat.tex_scale, uv) : mat.Kd; // assume albedo texture has been in linear color space
	float roughness	= (mat.roughness_tex)	? GetTexValFloat(mat.roughness_tex, mat.tex_scale, uv): mat.roughness;
	float metallic	= (mat.metallic_tex)	? GetTexValFloat(mat.metallic_tex, mat.tex_scale, uv) : mat.metallic;
	float3 specular	= (mat.ks_tex)			? GetTexFloat4ValFloat3(mat.ks_tex, mat.tex_scale, uv) : mat.Ks;
	
	// Get direction inner product
	float NdV = Dot(world_normal, -ray_dir);

	float3 Le = make_float3(0.f); // TODO: Add Emisions from hit point if needed

	// ---------------------------------------------------------- //
	// ---- Follow the next event estimation (NEE) framework ---- //
	// ---------------------------------------------------------- //

	// ---- Compute direct illumination from lights ---- //

	float NdH = 0.f, VdH = 0.f, NdL = 0.f;
	float3 halfway = make_float3(0.f);
	
	//  Uniformly sample a light source
	// if (cntxt_params.num_lights > 0) {
	// unsigned int light_idx = (unsigned int)(curand_uniform(&prd_camera->rng) * params.num_lights);
	// TODO: Use better importance random sampling for light selection,
	// where importance is proportional to light luminance [cd/m^2/sr] at the hit point
	for (unsigned int light_idx = 0; light_idx < cntxt_params.num_lights; light_idx++) {
		// float prob_choose_light = 1.0f / params.num_lights;
		ChOptixLight light = cntxt_params.lights[light_idx];
		
		// Check visibility to light source
		LightSample light_sample;
		light_sample.hitpoint = hit_point;
		light_sample.wo = -ray_dir;
		light_sample.n = world_normal;
		if(CheckVisibleAndSampleLight(cntxt_params, light, light_sample, prd_camera)) {
			halfway = normalize(light_sample.dir - ray_dir);
			NdH = Dot(world_normal, halfway);
			VdH = Dot(-ray_dir, halfway); // dot(V, H) = dot(H, L), since H is halfway between L and V
			// Compute direct lighting
			prd_camera->color = (light_sample.L
								* PrincipledBRDF(albedo, roughness, metallic, specular, 1.0f, mat.use_specular_workflow, NdV, light_sample.NdL, NdH, VdH))
								/ (light_sample.pdf);
			// prd_camera->color = {0., 1.0, 0.}; // debug
		}
		else {
			prd_camera->color = make_float3(0.f, 0.f, 0.f);
		}
	}
	// }
	
	// ---- Compute indirect illumination from the next ray ---- //
	
	if (prd_camera->depth + 1 < params.max_depth) {
		
		// printf("Next ray!\n");
		int weight_idx_start = 2, weight_idx_end = 2;
		float weights[2] = {0.f, 0.f}; // [diffuse weight, specular weight]
		float diffuse_prob = 0.;
		// int weight_idx = 2;
		if (prd_camera->use_gi) {
			// bsdf_type = BSDFType::SPECULAR; // debug
			if (bsdf_type == BSDFType::DIFFUSE) {
				weight_idx_start = 0;
				weight_idx_end = 1;
				weights[0] = 1.f; // diffuse weight
				weights[1] = 0.f; // specular weight
				// diffuse_prob = 1.f;
				// weight_idx = 0;
			}
			else if (bsdf_type == BSDFType::SPECULAR) {
				weight_idx_start = 1;
				weight_idx_end = 2;
				weights[0] = 0.f; // diffuse weight
				weights[1] = 1.f; // specular weight
				// diffuse_prob = 0.f;
				// weight_idx = 1;
			}
			else {
				diffuse_prob = 0.5f;
				
				// weight_idx_start = 0;
				// weight_idx_end = 2;
				// weights[0] = 0.5f; // diffuse weight
				// weights[1] = 0.5f; // specular weight
				
				// Randomly choose diffuse or specular reflection based on weights. weight_idx = 0: diffuse, 1: specular
				// int weight_idx = 0;
				weight_idx_start = static_cast<int>((curand_uniform(&prd_camera->rng) <= diffuse_prob) ? 0 : 1);;
				weight_idx_end = weight_idx_start + 1;
				weights[weight_idx_start] = 1.f;
			}
			// weight_idx = 0;
		}
		else {
			weight_idx_start = 1;
			weight_idx_end = 2;
			weights[0] = 0.f; // diffuse weight
			weights[1] = 1.f; // specular weight
			// weight_idx = 1;
		}

		// debug
		// weight_idx_start = 1;
		// weight_idx_end = 2;
		// weights[0] = 0.f; // diffuse weight
		// weights[1] = 1.f; // specular weight

		for (int weight_idx = weight_idx_start; weight_idx < weight_idx_end; ++weight_idx) {
			float3 next_dir;
			// Diffuse reflected ray
			if (weight_idx == 0) {
				// Just use cosine sampling to sample hemisphere for next ray
				next_dir = SampleCosineHemisphereDir(curand_uniform(&prd_camera->rng), curand_uniform(&prd_camera->rng), world_normal);
			}
			// Specular reflected ray
			else if (weight_idx == 1) {
				next_dir = normalize(reflect(ray_dir, world_normal));
			}
			else {
				prd_camera->color = make_float3(0., 0., 0.); // debug
				return;
			}
			
			NdL = Dot(world_normal, next_dir);
			float3 halfway = normalize(next_dir - ray_dir);
			float NdH = Dot(world_normal, halfway);
			float VdH = Dot(-ray_dir, halfway);  // Same as LdH
			float sampling_pdf = NdL / CUDART_PI_F; // cosine hemisphere pdf
			float3 this_contrib_to_pixel = PrincipledBRDF(albedo, roughness, metallic, specular, 1.0f, mat.use_specular_workflow, NdV, NdL, NdH, VdH);
			// float3 this_contrib_to_pixel = make_float3(1.f) * NdL;
			
			// LEGACY: magic heuristic mirror correction for specular reflection
			if (weight_idx == 1) 
				this_contrib_to_pixel = this_contrib_to_pixel * (1.f - roughness) * (1.f - roughness) * metallic * metallic; 
			
			float3 next_contrib_to_pixel = prd_camera->contrib_to_pixel * this_contrib_to_pixel;
			
			// Go through Russian roulette (RR)
			if (prd_camera->depth > 3 && fmaxf(next_contrib_to_pixel) < 0.1f) {
				RussianRoulette(prd_camera->rng, next_contrib_to_pixel);
			}

			// Determine if terminating the next ray
			if (luminance(next_contrib_to_pixel) > params.importance_cutoff) {
			// if (fmaxf(next_contrib_to_pixel) > 0.01f) {
				// Trace next ray
				PerRayData_camera prd_reflection = DefaultCameraPRD();
				prd_reflection.integrator = prd_camera->integrator;
				prd_reflection.rng = prd_camera->rng;
				prd_reflection.depth = prd_camera->depth + 1;
				prd_reflection.use_gi = prd_camera->use_gi;
				prd_reflection.use_fog = prd_camera->use_fog;
				prd_reflection.contrib_to_pixel = next_contrib_to_pixel;
				unsigned int opt1, opt2;
				pointer_as_ints(&prd_reflection, opt1, opt2);
				unsigned int raytype = static_cast<unsigned int>(RayType::CAMERA_RAY_TYPE);
				optixTrace(
					cntxt_params.root,          // The scene traversable handle (OptixTraversableHandle); basically the top-level acceleration structure (TLAS).
					hit_point,					// origin of the traced ray
					next_dir,           		// direction of the traced ray
					cntxt_params.scene_epsilon, // minimum intersection distance to avoid self-intersection (“shadow acne”)
					1e16f,          			// A very large max distance (effectively “infinite” for the scene scale)
					optixGetRayTime(),          // time value for launching this ray
					OptixVisibilityMask(1),     // Only intersects geometry whose instance mask matches 1
					OPTIX_RAY_FLAG_NONE,		// terminate on first hit is ideal for occlusion rays
					0,                          // SBT offset (used when you have multiple SBT records for the same ray type). It selects the first “ray type slot”
					1,                          // SBT stride (used when you have multiple SBT records for the same ray type). It usually means “one ray type stride”
					0,                          // missSBTIndex. It selects the first miss program
					opt1,                       // Final payloads; the per-ray data pointer (first 32 bits); optixGetPayload_0() = opt1
					opt2,                       // Final payloads; the per-ray data pointer (second 32 bits); optixGetPayload_1() = opt2
					raytype                     // The ray type index (used when you have multiple ray types, e.g., radiance rays, shadow rays, etc.)
				);
				
				// prd_camera->color += prd_reflection.color;
				prd_camera->color += weights[weight_idx] * (prd_reflection.color * this_contrib_to_pixel) / (sampling_pdf);
				// prd_camera->color += (prd_reflection.color * this_contrib_to_pixel) / (sampling_pdf)/* / (fabsf(diffuse_prob - 0.5) + 0.5)*/;
				
			}
		}
	}
	

	// Add ambient light response
	prd_camera->color += cntxt_params.ambient_light_color * albedo;
	// prd_camera->color = albedo; // debug

	// Account for fog
    AddFogEffect(prd_camera, cntxt_params, ray_dist);

	// Collect albedo and world_normal for OptiX denoiser for the first hits
    if (prd_camera->depth == 2) {
		prd_camera->albedo = albedo;  // Might change
		prd_camera->normal = world_normal;
	}
}

#endif  // CAMERA_PATH_SHADER_CU