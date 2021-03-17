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

rtDeclareVariable(PerRayData_radar, prd_radar, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
rtDeclareVariable(float, scene_epsilon, , );
rtDeclareVariable(float, max_scene_distance, , );
rtDeclareVariable(rtObject, root_node, , );

rtDeclareVariable(float, start_time, , );  // launch time for the sensor
rtDeclareVariable(float, end_time, , );    // launch time for the sensor

// radar parameters
rtDeclareVariable(float, hFOV, , );            // lidar horizontal field of view
rtDeclareVariable(float, max_vert_angle, , );  // lidar vertical field of view
rtDeclareVariable(float, min_vert_angle, , );  // lidar vertical field of view
rtDeclareVariable(float, max_distance, , );    // lidar maximum distance
rtDeclareVariable(float, clip_near, , );  // lidar minimum distance -> for use when lidar is placed within its casing

rtDeclareVariable(float3, origin_0, , );  // origin at time 0
rtDeclareVariable(float3, origin_1, , );  // origin at time 1
rtDeclareVariable(float4, rot_0, , );     // rotation at time 0 (no rotation is x forward, y left, x up)
rtDeclareVariable(float4, rot_1, , );     // rotation at time 1 (no rotation is x forward, y left, x up)

rtBuffer<float2, 2> output_buffer;  // byte version
rtBuffer<float> noise_buffer;



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
    optix::Ray ray(ray_origin, ray_direction, RADAR_RAY_TYPE, clip_near, max_distance);

    // set the ray pay load
    PerRayData_radar prd_radar = make_radar_data(0, 0, 0);

    // launch the ray
    const float current_time = start_time + (end_time - start_time) * (launch_index.x / (float)screen.x);  // rnd(seed);

    rtTrace(root_node, ray, current_time, prd_radar, RT_RAY_FLAG_DISABLE_ANYHIT);

    // set the output buffer to be what is returned in the payload
    output_buffer[launch_index] = make_float2(prd_radar.range, prd_radar.rcs);
}


//rtDeclareVariable(PerRayData_radar, prd_radar, rtPayload, );
//rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
//rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
//rtDeclareVariable(rtObject, root_node, , );
//
//rtDeclareVariable(float, start_time, , );
//rtDeclareVariable(float, end_time, , );
//
//// radar parameters
//rtDeclareVariable(float, hFOV, , ); // radar horizontal field of view
//rtDeclareVariable(float, min_vert_angle, , ); // radar vertical field of view
//rtDeclareVariable(float, max_vert_angle, , ); // radar vertical field of view
//rtDeclareVariable(float, max_distance, , ); // radar maximum distance
//rtDeclareVariable(float, clip_near, , ); // radar minimum distance -> for use when radar is placed within its casing
//
//
//rtDeclareVariable(float3, origin_0, , ); // origin at time 0
//rtDeclareVariable(float3, origin_1, , ); // origin at time 1
//rtDeclareVariable(float4, rot_0, , ); // rotation at time 0 (no rotation is x forward, y left, x up)
//rtDeclareVariable(float4, rot_1, , ); // rotation at time 1 (no rotation is x forward, y left, x up)
//
//
//rtBuffer<float2, 2> output_buffer; //byte version
//
//
//// This kernel is launched once for each ray
//RT_PROGRAM void spherical(){
//    size_t2 screen = output_buffer.size();
//
//    // set the ray direction based on the proportion of image the pixel is located at
//    float2 d = (make_float2(launch_index) + make_float2(0.5, 0.5)) / make_float2(screen) * 2.f - 1.f; 
//
//    float theta = d.x * hFOV / 2.0;
//    float phi = min_vert_angle + (d.y * .5 + .5) * (max_vert_angle - min_vert_angle);
//    float xy_proj = cos(phi);
//
//    float z = sin(phi);
//    float y = xy_proj * sin(theta);
//    float x = xy_proj * cos(theta);
//
//    const Quaternion q0(rot_0.y, rot_0.z, rot_0.w, rot_0.x);
//    const Quaternion q1(rot_1.y, rot_1.z, rot_1.w, rot_1.x);
//
//    Quaternion q_current = nlerp(q0, q1, (launch_index.x / (float)screen.x));
//    
//    Matrix4x4 basis;
//
//    q_current.toMatrix(basis.getData());
//    float3 forward = make_float3(basis[0], basis[4], basis[8]);
//    float3 left = make_float3(basis[1], basis[5], basis[9]);
//    float3 up = make_float3(basis[2], basis[6], basis[10]);
//
//    float3 ray_origin = lerp(origin_0, origin_1, (launch_index.x / (float)screen.x));
//    float3 ray_direction = normalize(forward * x + left * y + up * z);
//
//    // create a ray based on the calculated parameters
//    optix::Ray ray(ray_origin, ray_direction, RADAR_RAY_TYPE, clip_near, max_distance);
//
//    // set the ray pay load
//    PerRayData_radar prd_radar = make_radar_data(0, 0);
//
//    // launch the ray
//    const float current_time = start_time + (end_time - start_time) * (launch_index.x / (float)screen.x);
//    rtTrace(root_node, ray, current_time, prd_radar, RT_RAY_FLAG_DISABLE_ANYHIT);
//
//    // set the output buffer to be what is returnin in the payload
//    output_buffer[launch_index] = make_float2(prd_radar.range, prd_radar.rcs);
//
//    if (prd_radar.range > 0){
//        printf("hello\n");
//    } else {
//        printf("sad\n");
//    }
//}