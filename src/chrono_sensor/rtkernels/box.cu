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
 #define NOMINMAX
#endif

#include <optix.h>
#include <optixu/optixu_aabb_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
using namespace optix;

rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );
rtDeclareVariable(float3, tangent_vector, attribute tangent_vector, );
rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float2, texcoord, attribute texcoord, );

static __device__ float3 box_normal(float t, float3 t0, float3 t1) {
    float3 normal_pos = make_float3(t == t0.x ? 1 : 0, t == t0.y ? 1 : 0, t == t0.z ? 1 : 0);
    float3 normal_neg = make_float3(t == t1.x ? 1 : 0, t == t1.y ? 1 : 0, t == t1.z ? 1 : 0);
    return normal_neg - normal_pos;
}

RT_PROGRAM void box_intersect(int) {
    // calculate potential intersections with the box
    float3 t0 = (make_float3(-.5f) - ray.origin) / ray.direction;
    float3 t1 = (make_float3(.5f) - ray.origin) / ray.direction;
    float3 near = fminf(t0, t1);
    float3 far = fmaxf(t0, t1);
    // dist_near and dist_far are the distances to the potential intsection points
    float dist_near = fmaxf(near);
    float dist_far = fminf(far);

    // check if near is less than far
    if (dist_near <= dist_far) {
        float3 p = make_float3(0);
        bool check_second = true;
        if (rtPotentialIntersection(dist_near)) {
            shading_normal = box_normal(dist_near, t0, t1);
            p = ray.origin + dist_near * ray.direction;

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

            if (rtReportIntersection(0))
                check_second = false;
        }
        if (check_second) {
            if (rtPotentialIntersection(dist_far)) {
                shading_normal = box_normal(dist_far, t0, t1);
                p = ray.origin + dist_far * ray.direction;

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
                rtReportIntersection(0);
            }
        }
    }
}

RT_PROGRAM void box_bounds(int, float result[6]) {
    optix::Aabb* aabb = (optix::Aabb*)result;
    aabb->set(make_float3(-.5f), make_float3(.5f));
}
