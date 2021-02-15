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

rtDeclareVariable(PerRayData_lidar, prd_lidar, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
rtDeclareVariable(float, scene_epsilon, , );
rtDeclareVariable(float, max_scene_distance, , );
rtDeclareVariable(rtObject, root_node, , );

rtDeclareVariable(float, start_time, , );  // launch time for the sensor
rtDeclareVariable(float, end_time, , );    // launch time for the sensor

// lidar parameters
rtDeclareVariable(float, hFOV, , );            // lidar horizontal field of view
rtDeclareVariable(float, max_vert_angle, , );  // lidar vertical field of view
rtDeclareVariable(float, min_vert_angle, , );  // lidar vertical field of view
rtDeclareVariable(float, max_distance, , );    // lidar maximum distance
rtDeclareVariable(float, clip_near, , );  // lidar minimum distance -> for use when lidar is placed within its casing

rtDeclareVariable(float3, origin_0, , );  // origin at time 0
rtDeclareVariable(float3, origin_1, , );  // origin at time 1
rtDeclareVariable(float4, rot_0, , );     // rotation at time 0 (no rotation is x forward, y left, x up)
rtDeclareVariable(float4, rot_1, , );     // rotation at time 0 (no rotation is x forward, y left, x up)

rtBuffer<float2, 2> output_buffer;  // byte version
rtBuffer<float> noise_buffer;

// for beam divergence and multi-sample kernel
rtDeclareVariable(float, divergence_angle, , );  // lidar beam divergence
rtDeclareVariable(int, ray_samples, , );         // samples per beam

// This kernel is launched once for each pixel in the image
RT_PROGRAM void spherical() {
    size_t2 screen = output_buffer.size();

    // set the ray direction based on the proportion of image the pixel is located at
    float2 d = (make_float2(launch_index) + make_float2(0.5, 0.5)) / make_float2(screen) * 2.f - 1.f;  //[-1,1]

    float theta = d.x * hFOV / 2.0;
    float phi = min_vert_angle + (d.y * .5 + .5) * (max_vert_angle - min_vert_angle);
    float xy_proj = cos(phi);

    float z = sin(phi);
    float y = xy_proj * sin(theta);
    float x = xy_proj * cos(theta);

    const Quaternion q0(rot_0.y, rot_0.z, rot_0.w, rot_0.x);
    const Quaternion q1(rot_1.y, rot_1.z, rot_1.w, rot_1.x);

    Quaternion q_current = nlerp(q0, q1, (launch_index.x / (float)screen.x));

    Matrix4x4 basis;
    q_current.toMatrix(basis.getData());
    float3 forward = make_float3(basis[0], basis[4], basis[8]);  // TODO: calcualte from quaternion
    float3 left = make_float3(basis[1], basis[5], basis[9]);
    float3 up = make_float3(basis[2], basis[6], basis[10]);

    float3 ray_origin = lerp(origin_0, origin_1, (launch_index.x / (float)screen.x));
    float3 ray_direction = normalize(forward * x + left * y + up * z);

    // create a ray based on the calculated parameters
    optix::Ray ray(ray_origin, ray_direction, LIDAR_RAY_TYPE, clip_near, max_distance);

    // set the ray pay load
    PerRayData_lidar prd_lidar = make_lidar_data(0, 1.f, 0);

    // launch the ray
    const float current_time = start_time + (end_time - start_time) * (launch_index.x / (float)screen.x);  // rnd(seed);

    rtTrace(root_node, ray, current_time, prd_lidar, RT_RAY_FLAG_DISABLE_ANYHIT);

    // set the output buffer to be what is returned in the payload
    output_buffer[launch_index] = make_float2(prd_lidar.range, prd_lidar.intensity);
}

RT_PROGRAM void multi_sample() {
    size_t2 screen = output_buffer.size();

    float div_angle = divergence_angle;
    int sample_radius = ray_samples;

    int2 global_beam_dims = make_int2(screen.x / (sample_radius * 2 - 1), screen.y / (sample_radius * 2 - 1));
    int2 local_beam_dims = make_int2(sample_radius * 2 - 1, sample_radius * 2 - 1);

    // index of center of beam
    int beam_index_x = launch_index.x / (sample_radius * 2 - 1);
    int beam_index_y = launch_index.y / (sample_radius * 2 - 1);
    float2 beam_id_fraction =
        (make_float2(beam_index_x, beam_index_y) + make_float2(0.5, 0.5)) / make_float2(global_beam_dims) * 2.f -
        1.f;  //[-1,1]

    // theta and phi for beam center
    float beam_theta = beam_id_fraction.x * hFOV / 2.0;
    float beam_phi = min_vert_angle + (beam_id_fraction.y * .5 + .5) * (max_vert_angle - min_vert_angle);

    // index of local ray in beam
    int local_ray_index_x = launch_index.x % (sample_radius * 2 - 1);
    int local_ray_index_y = launch_index.y % (sample_radius * 2 - 1);
    float2 local_ray_id_fraction = (make_float2(local_ray_index_x, local_ray_index_y) + make_float2(0.5, 0.5)) /
                                       make_float2(local_beam_dims) * 2.f -
                                   1.f;  //[-1,1]

    // relative theta and phi for local ray in beam
    float local_ray_theta = local_ray_id_fraction.x * div_angle / 2.0;
    float local_ray_phi = local_ray_id_fraction.y * div_angle / 2.0;

    // carry on ray-tracing per ray
    float theta = beam_theta + local_ray_theta;
    float phi = beam_phi + local_ray_phi;

    float xy_proj = cos(phi);

    float z = sin(phi);
    float y = xy_proj * sin(theta);
    float x = xy_proj * cos(theta);

    const Quaternion q0(rot_0.y, rot_0.z, rot_0.w, rot_0.x);
    const Quaternion q1(rot_1.y, rot_1.z, rot_1.w, rot_1.x);

    Quaternion q_current = nlerp(q0, q1, (beam_index_x / (float)global_beam_dims.x));

    Matrix4x4 basis;
    q_current.toMatrix(basis.getData());
    float3 forward = make_float3(basis[0], basis[4], basis[8]);  // TODO: calcualte from quaternion
    float3 left = make_float3(basis[1], basis[5], basis[9]);
    float3 up = make_float3(basis[2], basis[6], basis[10]);

    float3 ray_origin = lerp(origin_0, origin_1, (beam_index_x / (float)global_beam_dims.x));
    float3 ray_direction = normalize(forward * x + left * y + up * z);

    // create a ray based on the calculated parameters
    optix::Ray ray(ray_origin, ray_direction, LIDAR_RAY_TYPE, clip_near, max_distance);

    // set the ray pay load
    PerRayData_lidar prd_lidar = make_lidar_data(0, 1.f, 0);

    const float current_time =
        start_time + (end_time - start_time) * (beam_index_x / (float)global_beam_dims.x);  // rnd(seed);

    // launch the ray
    rtTrace(root_node, ray, current_time, prd_lidar, RT_RAY_FLAG_DISABLE_ANYHIT);

    // set the output buffer to be what is returned in the payload
    output_buffer[launch_index] = make_float2(prd_lidar.range, prd_lidar.intensity);
}
