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
// Authors: Asher Elmquist, Han Wang
// =============================================================================
//
// =============================================================================

#ifndef CHOPTIXDEFINITIONS_H
#define CHOPTIXDEFINITIONS_H

#include <vector_types.h>
#include <optix_types.h>
#include <cuda_runtime_api.h>
#include <curand_kernel.h>

#include <cuda_fp16.h>
struct __align__(8) half4 {
    __half x;
    __half y;
    __half z;
    __half w;
};

enum RayType {
    CAMERA_RAY_TYPE = 0,  // camera rays
    SHADOW_RAY_TYPE = 1,  // shadow rays
    LIDAR_RAY_TYPE = 2,   // lidar rays
    RADAR_RAY_TYPE = 3,   // radar rays
    RAY_TYPE_COUNT        // use this enum to keep track of number of ray types
};

struct PointLight {
    float3 pos;
    float3 color;
    float max_range;
};

struct LidarMissParameters {
    float default_range;
    float default_intensity;
};

struct RadarMissParameters {
    float default_range;
    float default_rcs;
};

enum class BackgroundMode {
    SOLID_COLOR,     // solid color
    GRADIENT,        // gradient used for upper hemisphere
    ENVIRONMENT_MAP  // image used for spherical sky map
};

struct CameraMissParameters {
    BackgroundMode mode;
    float3 color_zenith;
    float3 color_horizon;
    cudaTextureObject_t env_map;
};

union MissParameters {
    CameraMissParameters camera_miss;
    LidarMissParameters lidar_miss;
    RadarMissParameters radar_miss;
};

struct CameraParameters {
    float hFOV;
    half4* frame_buffer;
    bool use_gi;                // whether to use global illumination
    half4* albedo_buffer;       // only initialized if using global illumination
    half4* normal_buffer;       // only initialized if using global illumination (screenspace normal)
    curandState_t* rng_buffer;  // only initialized if using global illumination
};

enum class LidarBeamShape {
    RECTANGULAR,  // rectangular beam
    ELLIPTICAL    // elliptical beam
};

struct LidarParameters {
    float max_vert_angle;
    float min_vert_angle;
    float hFOV;
    float max_distance;
    float clip_near;
    unsigned short sample_radius;
    LidarBeamShape beam_shape;
    float horiz_div_angle;
    float vert_div_angle;
    float2* frame_buffer;
};

struct RadarParameters {
    float max_vert_angle;
    float min_vert_angle;
    float hFOV;
    float max_distance;
    float clip_near;
    float horiz_div_angle;
    float vert_div_angle;
    float2* frame_buffer;
};

struct RaygenParameters {
    float t0;
    float t1;
    float3 pos0;
    float4 rot0;
    float3 pos1;
    float4 rot1;
    union {
        CameraParameters camera;
        LidarParameters lidar;
        RadarParameters radar;
    } specific;
};

struct __align__(16) MeshParameters {
    float4* vertex_buffer;
    float4* normal_buffer;
    float2* uv_buffer;
    uint4* vertex_index_buffer;
    uint4* normal_index_buffer;
    uint4* uv_index_buffer;
    unsigned int* mat_index_buffer;
    float pad;  // pads to 64 bytes
};

struct __align__(16) MaterialParameters {  // TODO: pad for alignment
    float3 Kd;
    float3 Ks;
    float fresnel_exp;
    float fresnel_min;
    float fresnel_max;
    float transparency;  // will also be the shadow attenuation
    float roughness;
    float metallic;
    float lidar_intensity;
    float radar_backscatter;
    int use_specular_workfloat;
    cudaTextureObject_t kd_tex;
    cudaTextureObject_t kn_tex;
    cudaTextureObject_t ks_tex;
    cudaTextureObject_t metallic_tex;
    cudaTextureObject_t roughness_tex;
    cudaTextureObject_t opacity_tex;
};

struct ContextParameters {
    PointLight* lights;
    int num_lights;
    float3 ambient_light_color;
    int max_depth;
    float scene_epsilon;
    float importance_cutoff;
    OptixTraversableHandle root;
    MaterialParameters* material_pool;
    MeshParameters* mesh_pool;
};

struct MaterialRecordParameters {
    unsigned int material_pool_id;
    unsigned int mesh_pool_id;
};

struct PerRayData_camera {
    float3 color;
    float contrib_to_first_hit;
    curandState_t rng;  // only valid if use_gi is true
    int depth;
    bool use_gi;
    float3 albedo;
    float3 normal;
};

struct PerRayData_shadow {
    float3 attenuation;
    int depth;
    float ramaining_dist;
};

struct PerRayData_lidar {
    float range;
    float intensity;
};

struct PerRayData_radar {
    float range;
    float rcs;
};

#endif
