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
// RT kernels for box geometries
//
// =============================================================================

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include "chrono_sensor/optix/shaders/device_utils.h"

static __device__ float3 box_normal(float t, float3 t0, float3 t1) {
    float3 normal_pos = make_float3(t == t0.x ? 1 : 0, t == t0.y ? 1 : 0, t == t0.z ? 1 : 0);
    float3 normal_neg = make_float3(t == t1.x ? 1 : 0, t == t1.y ? 1 : 0, t == t1.z ? 1 : 0);
    return normal_neg - normal_pos;
}

extern "C" __global__ void __intersection__box_intersect() {
    const float3 ray_orig = optixGetObjectRayOrigin();
    const float3 ray_dir = optixGetObjectRayDirection();
    const float ray_tmin = optixGetRayTmin();
    const float ray_tmax = optixGetRayTmax();

    // calculate potential intersections with the box
    float3 t0 = (make_float3(-.5f) - ray_orig) / ray_dir;
    float3 t1 = (make_float3(.5f) - ray_orig) / ray_dir;
    float3 near = fminf(t0, t1);
    float3 far = fmaxf(t0, t1);
    // dist_near and dist_far are the distances to the potential intsection points
    float dist_near = fmaxf(near);
    float dist_far = fminf(far);

    // check if near is less than far
    if (dist_near <= dist_far) {
        float3 p = make_float3(0);

        if (dist_near > ray_tmin && dist_near < ray_tmax) {
            float3 shading_normal = box_normal(dist_near, t0, t1);
            float2 texcoord;
            float3 tangent_vector;
            p = ray_orig + dist_near * ray_dir;

            if (abs(shading_normal.x) > 0.5) {
                texcoord = make_float2((p.y + 0.5), (p.z + 0.5) * shading_normal.x);
                tangent_vector = make_float3(0, 1, 0);
            } else if (abs(shading_normal.y) > 0.5) {
                texcoord = make_float2((p.x + 0.5), -(p.z + 0.5) * shading_normal.y);
                tangent_vector = make_float3(1, 0, 0);
            } else {
                texcoord = make_float2((p.x + 0.5), (p.y + 0.5) * shading_normal.z);
                tangent_vector = make_float3(1, 0, 0);
            }

            optixReportIntersection(
                dist_near, 0, reinterpret_cast<unsigned int&>(shading_normal.x),
                reinterpret_cast<unsigned int&>(shading_normal.y), reinterpret_cast<unsigned int&>(shading_normal.z),
                reinterpret_cast<unsigned int&>(texcoord.x), reinterpret_cast<unsigned int&>(texcoord.y),
                reinterpret_cast<unsigned int&>(tangent_vector.x), reinterpret_cast<unsigned int&>(tangent_vector.y),
                reinterpret_cast<unsigned int&>(tangent_vector.z));
        } else if (dist_far > ray_tmin && dist_far < ray_tmax) {
            float3 shading_normal = box_normal(dist_far, t0, t1);
            float2 texcoord;
            float3 tangent_vector;
            p = ray_orig + dist_far * ray_dir;

            // calculate uvs and tangent vector
            if (abs(shading_normal.x) > 0.5) {
                texcoord = make_float2((p.y + 0.5), (p.z + 0.5) * shading_normal.x);
                tangent_vector = make_float3(0, 1, 0);
            } else if (abs(shading_normal.y) > 0.5) {
                texcoord = make_float2((p.x + 0.5), -(p.z + 0.5) * shading_normal.y);
                tangent_vector = make_float3(1, 0, 0);
            } else {
                texcoord = make_float2((p.x + 0.5), (p.y + 0.5) * shading_normal.z);
                tangent_vector = make_float3(1, 0, 0);
            }
            optixReportIntersection(
                dist_far, 0, reinterpret_cast<unsigned int&>(shading_normal.x),
                reinterpret_cast<unsigned int&>(shading_normal.y), reinterpret_cast<unsigned int&>(shading_normal.z),
                reinterpret_cast<unsigned int&>(texcoord.x), reinterpret_cast<unsigned int&>(texcoord.y),
                reinterpret_cast<unsigned int&>(tangent_vector.x), reinterpret_cast<unsigned int&>(tangent_vector.y),
                reinterpret_cast<unsigned int&>(tangent_vector.z));
        }
    }
}
