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
// RT kernels for sphere geometries
//
// =============================================================================
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"

__device__ __inline__ void check_ends(const float3& ray_orig,
                                      const float3& ray_dir,
                                      const float& ray_tmin,
                                      const float& ray_tmax) {
    float t = (.5 - ray_orig.z) / ray_dir.z;
    float3 p = ray_orig + ray_dir * t;

    if (p.x * p.x + p.y * p.y - 1 < 0 && t > ray_tmin && t < ray_tmax) {
        float3 shading_normal = make_float3(0, 1, 0);
        float3 tangent_vector = make_float3(-1, 0, 0);
        float2 texcoord = make_float2(-p.x / 2.f + .5f, -p.y / 2.f + .5f);
        optixReportIntersection(
            t,                                                  //
            0,                                                  //
            reinterpret_cast<unsigned int&>(shading_normal.x),  //
            reinterpret_cast<unsigned int&>(shading_normal.y),  //
            reinterpret_cast<unsigned int&>(shading_normal.z),  //
            reinterpret_cast<unsigned int&>(texcoord.x), reinterpret_cast<unsigned int&>(texcoord.y),
            reinterpret_cast<unsigned int&>(tangent_vector.x), reinterpret_cast<unsigned int&>(tangent_vector.y),
            reinterpret_cast<unsigned int&>(tangent_vector.z));
    }

    t = (-.5 - ray_orig.z) / ray_dir.z;
    p = ray_orig + ray_dir * t;
    if (p.x * p.x + p.y * p.y - 1 < 0 && t > ray_tmin && t < ray_tmax) {
        float3 shading_normal = make_float3(0, 1, 0);
        float3 tangent_vector = make_float3(1, 0, 0);
        float2 texcoord = make_float2(p.x / 2.f + .5f, p.y / 2.f + .5f);
        optixReportIntersection(
            t,                                                  //
            0,                                                  //
            reinterpret_cast<unsigned int&>(shading_normal.x),  //
            reinterpret_cast<unsigned int&>(shading_normal.y),  //
            reinterpret_cast<unsigned int&>(shading_normal.z),  //
            reinterpret_cast<unsigned int&>(texcoord.x), reinterpret_cast<unsigned int&>(texcoord.y),
            reinterpret_cast<unsigned int&>(tangent_vector.x), reinterpret_cast<unsigned int&>(tangent_vector.y),
            reinterpret_cast<unsigned int&>(tangent_vector.z));
    }
}

extern "C" __global__ void __intersection__cylinder_intersect() {
    const float3 ray_orig = optixGetObjectRayOrigin();
    const float3 ray_dir = optixGetObjectRayDirection();
    const float ray_tmin = optixGetRayTmin();
    const float ray_tmax = optixGetRayTmax();

    check_ends(ray_orig, ray_dir, ray_tmin, ray_tmax);

    float a = ray_dir.x * ray_dir.x + ray_dir.y * ray_dir.y;
    float b = 2 * (ray_dir.x * ray_orig.x + ray_dir.y * ray_orig.y);
    float c = ray_orig.x * ray_orig.x + ray_orig.y * ray_orig.y - 1;
    float det = b * b - 4 * a * c;

    if (det > 0) {
        const float dist_near = (-b - sqrtf(det)) / (2 * a);
        const float dist_far = (-b + sqrtf(det)) / (2 * a);

        if (dist_near <= dist_far) {
            const float3 p_near = ray_orig + ray_dir * dist_near;
            const float3 p_far = ray_orig + ray_dir * dist_far;

            if (dist_near > ray_tmin && dist_near < ray_tmax && p_near.z < .5 && p_near.z > -.5) {
                float3 shading_normal = p_near - make_float3(0, 0, p_near.z);
                float3 tangent_vector = make_float3(p_near.y, 0, -p_near.x);
                float2 texcoord = make_float2(atan2(p_near.y, p_near.x) / (2 * CUDART_PI_F), p_near.z * 0.5 + 0.5);
                optixReportIntersection(dist_near,                                          //
                                        0,                                                  //
                                        reinterpret_cast<unsigned int&>(shading_normal.x),  //
                                        reinterpret_cast<unsigned int&>(shading_normal.y),  //
                                        reinterpret_cast<unsigned int&>(shading_normal.z),  //
                                        reinterpret_cast<unsigned int&>(texcoord.x),
                                        reinterpret_cast<unsigned int&>(texcoord.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.x),
                                        reinterpret_cast<unsigned int&>(tangent_vector.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.z));
            } else if (dist_far > ray_tmin && dist_far < ray_tmax && p_far.z < .5 && p_far.z > -.5) {
                float3 shading_normal = p_far - make_float3(0, 0, p_far.z);
                float3 tangent_vector = make_float3(p_far.y, 0, -p_far.x);
                float2 texcoord = make_float2(atan2(p_far.y, p_far.x) / (2 * CUDART_PI_F), p_far.z * 0.5 + 0.5);
                optixReportIntersection(dist_far,                                           //
                                        0,                                                  //
                                        reinterpret_cast<unsigned int&>(shading_normal.x),  //
                                        reinterpret_cast<unsigned int&>(shading_normal.y),  //
                                        reinterpret_cast<unsigned int&>(shading_normal.z),  //
                                        reinterpret_cast<unsigned int&>(texcoord.x),
                                        reinterpret_cast<unsigned int&>(texcoord.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.x),
                                        reinterpret_cast<unsigned int&>(tangent_vector.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.z));
            }
        }
    }
}
