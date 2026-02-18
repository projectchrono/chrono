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
// Authors: Nevindu M. Batagoda, Bo-Hsun Chen
// =============================================================================
//
// Registration structs of device-side lights.
//
// =============================================================================

#ifndef CHRONO_SENSOR_OPTIX_LIGHT_STRUCTS_H
#define CHRONO_SENSOR_OPTIX_LIGHT_STRUCTS_H

#include <vector_types.h>
#include <optix_types.h>
#include <cuda_runtime_api.h>
#include <curand_kernel.h>
#include <cuda_fp16.h>

// #include "chrono/core/ChVector3.h"
// #include "chrono/assets/ChColor.h"

// using namespace chrono;

enum class LightType {
	POINT_LIGHT,
	SPOT_LIGHT,
	DIRECTIONAL_LIGHT,
	RECTANGLE_LIGHT,
	DISK_LIGHT,
	ENVIRONMENT_LIGHT,
	AREA_LIGHT
	// ---- Register Your Customized Light Here (type definition) ---- //
};

struct __device__ LightSample {
    float3 L;        // light luminance, [cd/m^2/sr/sec] or [W/m^2/sr]
    float3 dir;      // wi, direction from hit-point to light
    float3 wo;       // direction from hit-point to viewer
    float3 n;        // world_normal at hit-point
    float3 hitpoint; // world position of hit-point
    float NdL;       // dot(n, dir)
    float dist;      // distance from hit-point to light
    float pdf;       // PDF of the light sample. Ex: delta lights have PDF = 1, area lights have PDF = 1 / area.
};


// ---- Register Your Customized Light Here (define light data structs) ---- //

/// Environment light data struct
struct EnvironmentLightData {
    float intensity_scale;				// [1/1], scale factor for the intensity of the environment light
    // Extended parameters for importance sampling
	cudaTextureObject_t env_map;		// CUDA texture object for the environment map
	int width;							// [px], width of the environment map
    int height;							// [px], height of the environment map
    unsigned short* dev_cdf_lat;	// device pointer to the CDF for latitude sampling, size = height x 2 Bytes
    unsigned short* dev_cdf_lon;	// device pointer to the CDF for longitude sampling, size = height x width x 2 Bytes
};

/// Disk light data struct
struct DiskLightData {
	float3 light_dir;			// unit normal vector of the disk light
	float radius;				// [m], radius of the disk light
	float3 color;				// color intensity of the light
	float max_range;			// [m], distance range at which the light intensity falls to 1% of its maximum color intensity. If set to -1, follows inverse square law.
	bool const_color;			// whether to use constant color (no attenuation with distance)
	// Extended parameters
	float atten_scale;			// [1/1], attenuation scale based on max_range
	float area;					// [m^2], area of the disk light
};

/// Rectangle light data struct
struct RectangleLightData {
	float3 length_vec;		// [m], one edge vector of the rectangle light
	float3 width_vec;		// [m], the other edge vector of the rectangle light perpendicular to length_vec. Light direction is: length_vec x width_vec.
	float3 color;			// color intensity of the light
	float max_range;		// [m], distance range at which the light intensity falls to 1% of its maximum color intensity. If set to -1, follows inverse square law.
	bool const_color;		// whether to use constant color (no attenuation with distance)
	// Extended parameters
	float atten_scale;		// [1/1], attenuation scale based on max_range
	float area;				// [m^2], area of the rectangle light
	float3 light_dir;		// unit direction vector of the rectangle light (normal vector)
};

/// Directional light data struct
struct DirectionalLightData {
	float3 color;		// [W/sr/m^2], color radiance of the light
	float elevation;	// [rad], elevation angle of the directional light comes from
	float azimuth;		// [rad], azimuth angle of the directional light comes from
	// Extended parameters
	float3 light_dir;	// unit direction vector from the hit-point to the directional light
};

/// Spot light data struct
struct SpotLightData {
	float3 color;				// [cd/sec] or [W], color intensity of the light
	float max_range;			// [m], distance range at which the light intensity falls to 1% of its maximum color intensity. If set to -1, follows inverse square law.
	float3 light_dir;			// unit direction vector of the spotlight
	float angle_falloff_start;	// [rad], angle at which the spotlight starts to linearly fall off
	float angle_range;			// [rad], angle range of the spotlight falling off to zero.
	bool const_color;			// whether to use constant color (no attenuation with distance)
	// Extended parameters
	float atten_scale;			// [1/1], attenuation scale based on max_range
	float angle_atten_rate;		// [1/rad], angular attenuation rate from "angle_falloff_start" to "angle_range"
};

/// Point light data struct
struct PointLightData {
	float3 color;			// [cd/sec] or [W], color intensity of the light
	float max_range;		// [m], range at which the light intensity falls to 1% of its maximum color intensity. If set to -1, follows inverse square law.
	float atten_scale;		// [1/1], attenuation scale based on max_range
	bool const_color;		// whether to use constant color (no attenuation with distance)
};

/// Base light struct
struct ChOptixLight {
	LightType light_type;					// type of light
	bool delta;								// whether the light is a delta light source
	float3 pos; 							// [m], position of the light
	union {
		PointLightData point;				// point light specific parameters
		SpotLightData spot;					// spot light specific parameters
		DirectionalLightData directional;	// directional light specific parameters
		RectangleLightData rectangle;		// rectangle light specific parameters
		DiskLightData disk;					// disk light specific parameters
		EnvironmentLightData environment;	// environment light specific parameters
		// ---- Register Your Customized Light Here (register light data struct) ---- //
	} specific;								// specific parameters for different light types
};


// // Point light struct
// struct ChOptixPointLight : public ChOptixLight {
// 	ChOptixPointLight(float3 pos, float3 color, float max_range) 
// 	: ChOptixLight(
// 		LightType::POINT_LIGHT, true, pos, color, max_range,
// 		((max_range > 0) ? (0.01 * max_range * max_range) : 1.f)
// 	) {}
// };

/*
/// @brief Set the position of the light.
/// @param position The new position of the light.
void SetLightPos(ChOptixLight& light, const ChVector3f& position) {
	light.pos = {position.x(), position.y(), position.z()};
}

/// @brief Set the orientation of the light.
/// @param new_x_axis The new x-axis of the light's orientation.
/// @param new_y_axis The new y-axis of the light's orientation.
/// @param new_z_axis The new z-axis of the light's orientation.
void SetLightRot(
	ChOptixLight& light, const ChVector3f& new_x_axis, const ChVector3f& new_y_axis, const ChVector3f& new_z_axis
) {
	light.x_axis = {new_x_axis.x(), new_x_axis.y(), new_x_axis.z()};
	light.y_axis = {new_y_axis.x(), new_y_axis.y(), new_y_axis.z()};
	light.z_axis = {new_z_axis.x(), new_z_axis.y(), new_z_axis.z()};
}

/// @brief Set the color intensity of the light.
/// @param color The new color intensity of the light.
void SetLightColor(ChOptixLight& light, const ChColor& color) { light.color = {color.R, color.G, color.B}; }

// /// @brief  Set the maximum range of the light.
// /// @param new_max_range The new maximum range of the light.
// void SetLightMaxRange(ChOptixLight& light, const float& new_max_range) {
// 	light.max_range = new_max_range;
// 	// Calculate the attenuation scale of the light based on its maximum range.
// 	light.atten_scale = (new_max_range > 0) ? (0.01 * new_max_range * new_max_range) : 1.f;	
// }
*/

#endif  // CHRONO_SENSOR_OPTIX_LIGHT_STRUCTS_H