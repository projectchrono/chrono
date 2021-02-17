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
// RT kernels for tracing and measureing depth for a typical pinhole camera
//
// =============================================================================

#include <math_constants.h>
#include <optixu/optixu_aabb.h>
#include <optixu/optixu_quaternion_namespace.h>
#include <optixu/optixu_matrix_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"

using namespace optix;

rtDeclareVariable(PerRayData_camera, prd_camera, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
rtDeclareVariable(float, scene_epsilon, , );
rtDeclareVariable(float, max_scene_distance, , );
rtDeclareVariable(rtObject, root_node, , );
rtDeclareVariable(float3, default_color, , );
rtDeclareVariable(float, default_depth, , );
rtDeclareVariable(int, max_depth, , );  
// camera parameters
rtDeclareVariable(float, hFOV, , );        // camera horizontal field of view
rtDeclareVariable(float, start_time, , );  // launch time for the sensor
rtDeclareVariable(float, end_time, , );    // launch time for the sensor
rtDeclareVariable(float3, origin_0, , );   // origin at time 0
rtDeclareVariable(float3, origin_1, , );   // origin at time 1
rtDeclareVariable(float4, rot_0, , );      // rotation at time 0 (no rotation is x forward, y left, x up)
rtDeclareVariable(float4, rot_1, , );      // rotation at time 0 (no rotation is x forward, y left, x up)

// environment map
rtTextureSampler<float4, 2> environment_map;
rtDeclareVariable(int, has_environment_map, , );

rtBuffer<float4, 2> output_buffer;  // byte version
rtBuffer<float4, 2> gi_pass_normal_buffer;  // byte version
rtBuffer<float4, 2> gi_pass_albedo_buffer;  // byte version

rtBuffer<float> noise_buffer;

// temporary using Optix Example !!!!
static __host__ __device__ __inline__ unsigned int lcg(unsigned int& prev) {
    const unsigned int LCG_A = 1664525u;
    const unsigned int LCG_C = 1013904223u;
    prev = (LCG_A * prev + LCG_C);
    return prev & 0x00FFFFFF;
}
// Generate random float in [0, 1)
static __host__ __device__ __inline__ float rnd(unsigned int& prev) {
    return ((float)lcg(prev) / (float)0x01000000);
}

RT_PROGRAM void pinhole_gi_camera() {
    size_t2 screen = output_buffer.size();

    float rand = noise_buffer[screen.x * launch_index.y + launch_index.x];
    unsigned int seed = (unsigned int)(rand * 2147483648u);
    noise_buffer[screen.x * launch_index.y + launch_index.x] = sensor_rand(seed);

    const float current_time = start_time + (end_time - start_time) * rand;

    // set the ray direction based on the proportion of image the pixel is located at
    float2 d = (make_float2(launch_index) + make_float2(0.5, 0.5)) / make_float2(screen) * 2.f - 1.f;
    d.y *= (float)(screen.y) / (float)(screen.x);
    // origin of the camera is  0,0,0 for now

    const float h_factor = hFOV / CUDART_PI_F * 2.0;

    const Quaternion q0(rot_0.y, rot_0.z, rot_0.w, rot_0.x);
    const Quaternion q1(rot_1.y, rot_1.z, rot_1.w, rot_1.x);

    Quaternion q_current = nlerp(q0, q1, rand);

    Matrix4x4 basis;
    q_current.toMatrix(basis.getData());
    float3 forward = make_float3(basis[0], basis[4], basis[8]);  // TODO: calcualte from quaternion
    float3 left = make_float3(basis[1], basis[5], basis[9]);
    float3 up = make_float3(basis[2], basis[6], basis[10]);

    float3 ray_origin = lerp(origin_0, origin_1, rand);
    float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

    float3 accumColor = make_float3(0);
    float attenuation = 1;

    // Initialze per-ray data
    PerRayData_camera prd_camera = make_camera_data(make_float3(0), 1, 1, 1.f);
    prd_camera.seed = seed;
    prd_camera.origin = ray_origin;
    prd_camera.direction = ray_direction;
    // Each iteration is a segment of the ray path.  The closest hit will
    // return new segments to be traced here.
    for (;;) {
        prd_camera.color = make_float3(0);
        optix::Ray ray(ray_origin, ray_direction, CAMERA_RAY_TYPE, scene_epsilon, max_scene_distance);
        rtTrace(root_node, ray, current_time, prd_camera);

        // Russian roulette termination, as depth increase, more likely to attenuate
        if (prd_camera.depth >= 3) {
            if (rnd(prd_camera.seed) >= attenuation)
                break;
            attenuation *= 0.5;
        }
        
        prd_camera.depth++;
        accumColor += prd_camera.color; // Accumulate the color
        
        // early termination if the contribution is low
        if (fmaxf(prd_camera.contribution_to_firsthit) < 0.01f)
            break;

        // Update ray data for the next path segment
        ray_origin = prd_camera.origin;
        ray_direction = prd_camera.direction;
    }

    output_buffer[launch_index] = make_float4(accumColor, 1);
    gi_pass_normal_buffer[launch_index] = make_float4(prd_camera.normal, 1);// * 0.5f + 0.5f;
    gi_pass_albedo_buffer[launch_index] = make_float4(prd_camera.albedo, 1);
}

// This kernel is launched once for each pixel in the image
RT_PROGRAM void pinhole_camera() {
    size_t2 screen = output_buffer.size();

    float rand = noise_buffer[screen.x * launch_index.y + launch_index.x];
    unsigned int seed = (unsigned int)(rand * 2147483648u);
    noise_buffer[screen.x * launch_index.y + launch_index.x] = sensor_rand(seed);

    const float current_time = start_time + (end_time - start_time) * rand;

    // set the ray direction based on the proportion of image the pixel is located at
    float2 d = (make_float2(launch_index) + make_float2(0.5, 0.5)) / make_float2(screen) * 2.f - 1.f;
    d.y *= (float)(screen.y) / (float)(screen.x);
    // origin of the camera is  0,0,0 for now

    const float h_factor = hFOV / CUDART_PI_F * 2.0;

    const Quaternion q0(rot_0.y, rot_0.z, rot_0.w, rot_0.x);
    const Quaternion q1(rot_1.y, rot_1.z, rot_1.w, rot_1.x);

    Quaternion q_current = nlerp(q0, q1, rand);

    Matrix4x4 basis;
    q_current.toMatrix(basis.getData());
    float3 forward = make_float3(basis[0], basis[4], basis[8]);  // TODO: calcualte from quaternion
    float3 left = make_float3(basis[1], basis[5], basis[9]);
    float3 up = make_float3(basis[2], basis[6], basis[10]);

    float3 ray_origin = lerp(origin_0, origin_1, rand);
    float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

    // create a ray based on the calculated parameters
    optix::Ray ray(ray_origin, ray_direction, CAMERA_RAY_TYPE, scene_epsilon, max_scene_distance);
    // set the ray pay load
    PerRayData_camera prd_camera = make_camera_data(make_float3(0), 0, 1, 1.f);

    rtTrace(root_node, ray, current_time, prd_camera);
    output_buffer[launch_index] = make_float4(prd_camera.color, 1);
}

// This kernel is launched once for each pixel in the image
RT_PROGRAM void fov_lens_camera() {
    size_t2 screen = output_buffer.size();

    float rand = noise_buffer[screen.x * launch_index.y + launch_index.x];
    unsigned int seed = (unsigned int)(rand * 2147483648u);
    noise_buffer[screen.x * launch_index.y + launch_index.x] = sensor_rand(seed);

    const float current_time = start_time + (end_time - start_time) * rand;

    // set the ray direction based on the proportion of image the pixel is located at
    float2 d = (make_float2(launch_index) + make_float2(0.5, 0.5)) / make_float2(screen) * 2.f - 1.f;
    // rescale y direction to be proportional to x
    d.y *= (float)(screen.y) / (float)(screen.x);

    if (abs(d.x) > 1e-5 || abs(d.y) > 1e-5) {
        // float omega = c_hFOV * (tan(tan(c_hFOV / 2.0)) / tan(c_hFOV / 2.0));

        float r1 = sqrtf(d.x * d.x + d.y * d.y);

        // float r2 = tan(A * r1 * c_hFOV / 2.0) / (tan(c_hFOV / 2.0));
        float r2 = tanf(r1 * tanf(hFOV / 2.0)) / tanf(hFOV / 2.0);
        // float r2 = r1 / cos(c_hFOV / 2);

        float scaled_extent = tanf(tanf(hFOV / 2.0)) / tanf(hFOV / 2.0);

        // float y_max = 1.4142;

        d.x = d.x * (r2 / r1) / scaled_extent;
        d.y = d.y * (r2 / r1) / scaled_extent;
    }

    // origin of the camera is  0,0,0 for now
    float h_factor = hFOV / CUDART_PI_F * 2.0;

    const Quaternion q0(rot_0.y, rot_0.z, rot_0.w, rot_0.x);
    const Quaternion q1(rot_1.y, rot_1.z, rot_1.w, rot_1.x);

    Quaternion q_current = nlerp(q0, q1, rand);

    Matrix4x4 basis;
    q_current.toMatrix(basis.getData());
    float3 forward = make_float3(basis[0], basis[4], basis[8]);  // TODO: calcualte from quaternion
    float3 left = make_float3(basis[1], basis[5], basis[9]);
    float3 up = make_float3(basis[2], basis[6], basis[10]);

    float3 ray_origin = lerp(origin_0, origin_1, rand);
    float3 ray_direction = normalize(forward - d.x * left * h_factor + d.y * up * h_factor);

    // create a ray based on the calculated parameters
    optix::Ray ray(ray_origin, ray_direction, CAMERA_RAY_TYPE, scene_epsilon, max_scene_distance);

    // set the ray pay load
    PerRayData_camera prd_camera = make_camera_data(make_float3(0), 0, 1, 1.f);

    // launch the ray
    rtTrace(root_node, ray, current_time, prd_camera, RT_RAY_FLAG_DISABLE_ANYHIT);

    // set the output buffer to be what is returned in the payload
    output_buffer[launch_index] = make_float4(prd_camera.color, 1);
    // output_buffer[launch_index] = make_color(prd_camera.color);
}
