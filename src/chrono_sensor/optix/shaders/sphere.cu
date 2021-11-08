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

extern "C" __global__ void __intersection__sphere_intersect() {
    const float3 ray_orig = optixGetObjectRayOrigin();
    const float3 ray_dir = optixGetObjectRayDirection();
    const float ray_tmin = optixGetRayTmin();
    const float ray_tmax = optixGetRayTmax();

    // calculate the three components of quadratic equation that defines intertersection
    float a = Dot(ray_dir, ray_dir);
    float b = 2 * Dot(ray_dir, ray_orig);
    float c = Dot(ray_orig, ray_orig) - 1;

    float det = b * b - 4 * a * c;

    if (det > 0) {
        det = sqrtf(det);
        float dist_near = (-b - det) / (2 * a);
        float dist_far = (-b + det) / (2 * a);

        if (dist_near <= dist_far) {
            if (dist_near > ray_tmin && dist_near < ray_tmax) {
                float3 p = ray_orig + ray_dir * dist_near;
                float3 tangent_vector = make_float3(p.y, -p.x, 0);
                float2 texcoord = make_float2(atan2(p.x, p.y) / (2 * CUDART_PI_F) + 0.5, p.z * 0.5 + 0.5);

                optixReportIntersection(dist_near,                             //
                                        0,                                     //
                                        reinterpret_cast<unsigned int&>(p.x),  //
                                        reinterpret_cast<unsigned int&>(p.y),  //
                                        reinterpret_cast<unsigned int&>(p.z),  //
                                        reinterpret_cast<unsigned int&>(texcoord.x),
                                        reinterpret_cast<unsigned int&>(texcoord.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.x),
                                        reinterpret_cast<unsigned int&>(tangent_vector.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.z));
            } else if (dist_far > ray_tmin && dist_far < ray_tmax) {
                float3 p = ray_orig + ray_dir * dist_far;
                float3 tangent_vector = make_float3(p.y, -p.x, 0);
                float2 texcoord = make_float2(atan2(p.x, p.y) / (2 * CUDART_PI_F) + 0.5, p.z * 0.5 + 0.5);
                optixReportIntersection(dist_far,  //
                                        0,         //
                                        reinterpret_cast<unsigned int&>(p.x), reinterpret_cast<unsigned int&>(p.y),
                                        reinterpret_cast<unsigned int&>(p.z),  //
                                        reinterpret_cast<unsigned int&>(texcoord.x),
                                        reinterpret_cast<unsigned int&>(texcoord.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.x),
                                        reinterpret_cast<unsigned int&>(tangent_vector.y),
                                        reinterpret_cast<unsigned int&>(tangent_vector.z));
            }
        }
    }
}
