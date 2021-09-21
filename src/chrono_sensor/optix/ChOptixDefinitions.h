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
struct half4 {
    __half x;
    __half y;
    __half z;
    __half w;
};

enum RayType {
    CAMERA_RAY_TYPE = 0,       // camera rays
    SHADOW_RAY_TYPE = 1,       // shadow rays
    LIDAR_RAY_TYPE = 2,        // lidar rays
    RADAR_RAY_TYPE = 3,        // radar rays
    SEGMENTATION_RAY_TYPE = 4  // semantic camera rays
};

struct PointLight {
    float3 pos;
    float3 color;
    float max_range;
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

struct MissParameters {
    CameraMissParameters camera_miss;
};

struct CameraParameters {
    float hFOV;                 ///< horizontal field of view
    float gamma;                ///< camera's gamma value
    half4* frame_buffer;        ///< buffer of camera pixels
    bool use_gi;                // whether to use global illumination
    half4* albedo_buffer;       // only initialized if using global illumination
    half4* normal_buffer;       // only initialized if using global illumination (screenspace normal)
    curandState_t* rng_buffer;  // only initialized if using global illumination
};

struct SemanticCameraParameters {
    float hFOV;                 ///< horizontal field of view
    ushort2* frame_buffer;      ///< buffer of class and instance ids
    curandState_t* rng_buffer;  ///< only initialized if using global illumination
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

enum class RadarReturnMode { RETURN, TRACK };

struct RadarParameters {
    float max_vert_angle;
    float min_vert_angle;
    float hFOV;
    float max_distance;
    float clip_near;
    float horiz_div_angle;
    float vert_div_angle;
    float3 velocity;
    float* frame_buffer;
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
        SemanticCameraParameters segmentation;
        LidarParameters lidar;
        RadarParameters radar;
    } specific;
};

struct MeshParameters {              // pad to align 16 (swig doesn't support explicit alignment calls)
    float4* vertex_buffer;           // size 8
    float4* normal_buffer;           // size 8
    float2* uv_buffer;               // size 8
    uint4* vertex_index_buffer;      // size 8
    uint4* normal_index_buffer;      // size 8
    uint4* uv_index_buffer;          // size 8
    unsigned int* mat_index_buffer;  // size 8
    double pad;                      // size 8
};

struct MaterialParameters {             // pad to align 16 (swig doesn't support explicit alignment calls)
    float3 Kd;                          // size 12
    float3 Ks;                          // size 12
    float fresnel_exp;                  // size 4
    float fresnel_min;                  // size 4
    float fresnel_max;                  // size 4
    float transparency;                 // size 4
    float roughness;                    // size 4
    float metallic;                     // size 4
    float lidar_intensity;              // size 4
    float radar_backscatter;            // size 4
    int use_specular_workflow;          // size 4
    cudaTextureObject_t kd_tex;         // size 8
    cudaTextureObject_t kn_tex;         // size 8
    cudaTextureObject_t ks_tex;         // size 8
    cudaTextureObject_t metallic_tex;   // size 8
    cudaTextureObject_t roughness_tex;  // size 8
    cudaTextureObject_t opacity_tex;    // size 8
    unsigned short int class_id;        // size 2
    unsigned short int instance_id;     // size 2
    // float3 pad;                         // size 12
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
    float3 translational_velocity;
    float3 angular_velocity;
    float objectID;
};

struct PerRayData_camera {
    float3 color;
    float3 contrib_to_pixel;
    curandState_t rng;  // only valid if use_gi is true
    int depth;
    bool use_gi;
    float3 albedo;
    float3 normal;
};

struct PerRayData_semantic {
    unsigned short int class_id;
    unsigned short int instance_id;
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
    float3 velocity;
    float objectID;
};

#endif
