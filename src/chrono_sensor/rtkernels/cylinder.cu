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
 #define NOMINMAX
#endif

#include <optix.h>
#include <optixu/optixu_aabb_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
using namespace optix;

// rtDeclareVariable(float3, center, , );  // center of the cylinder
// rtDeclareVariable(float2, geom, , );    // radius and height of the cylinder
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );

rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float3, tangent_vector, attribute tangent_vector, );
rtDeclareVariable(float2, texcoord, attribute texcoord, );
rtDeclareVariable(int, has_texture, , );

static __device__ __inline__ bool check_ends(const float3& o, const float3& d) {
    float t = (.5 - o.y) / d.y;
    float3 p_circle = o + d * t;

    bool found_intersection = false;

    if (t > 0 && p_circle.x * p_circle.x + p_circle.z * p_circle.z - 1 < 0) {
        if (rtPotentialIntersection(t)) {
            shading_normal = make_float3(0, 1, 0);
            tangent_vector = make_float3(0, 0, 1);
            texcoord = make_float2(0.0f);  // TODO: compute the angle to use for the tex coordinate
            if (rtReportIntersection(0))
                found_intersection = true;
        }
    }

    t = (-.5 - o.y) / d.y;
    p_circle = o + d * t;
    if (t > 0 && p_circle.x * p_circle.x + p_circle.z * p_circle.z - 1 < 0) {
        if (rtPotentialIntersection(t)) {
            shading_normal = make_float3(0, -1, 0);
            tangent_vector = make_float3(0, 0, 1);
            texcoord = make_float2(0.0f);  // TODO: compute the angle to use for the tex coordinate
            if (rtReportIntersection(0))
                found_intersection = true;
        }
    }

    return found_intersection;
}

RT_PROGRAM void cylinder_intersect(int) {
    // calculate the three components of quadratic equation that defines intertersection
    float a = ray.direction.x * ray.direction.x + ray.direction.z * ray.direction.z;
    float b = 2 * (ray.direction.x * ray.origin.x + ray.direction.z * ray.origin.z);
    float c = ray.origin.x * ray.origin.x + ray.origin.z * ray.origin.z - 1;

    float det = b * b - 4 * a * c;

    // inefficient, but prevents strange issue with not detecting close end cap when looking straight down the cylinder
    check_ends(ray.origin, ray.direction);

    if (det > 1e-9) {
        float dist_near = (-b - sqrtf(det)) / (2 * a);

        // if (dist_near < dist_far) {
        bool check_second = true;
        if (dist_near > 0) {
            // if (rtPotentialIntersection(dist_near)) {
            float3 p = ray.origin + ray.direction * dist_near;
            if (p.y > .5 || p.y < -.5) {
            } else {
                // means we hit the side
                if (rtPotentialIntersection(dist_near)) {
                    shading_normal = p - make_float3(0, p.y, 0);
                    tangent_vector = make_float3(0, 0, 1);
                    texcoord = make_float2(0.0f);  // TODO: compute the angle to use for the tex coordinate
                    if (rtReportIntersection(0))
                        check_second = false;
                }
            }
        }
        if (check_second) {
            float dist_far = (-b + sqrtf(det)) / (2 * a);
            float3 p = ray.origin + ray.direction * dist_far;
            if (p.y > .5 || p.y < -.5) {
            } else {
                // // means we hit the side
                if (rtPotentialIntersection(dist_far)) {
                    shading_normal = p - make_float3(0, p.y, 0);
                    tangent_vector = make_float3(0, 0, 1);
                    texcoord = make_float2(0.0f);  // TODO: compute the angle to use for the tex coordinate
                    rtReportIntersection(0);
                }
            }
        }
    }
}

RT_PROGRAM void cylinder_bounds(int, float result[6]) {
    // const float3 edge_lengths = make_float3(geom.x, geom.y / 2.0, geom.x);

    optix::Aabb* aabb = (optix::Aabb*)result;
    aabb->set(make_float3(-1.f, -.5f, -1.f), make_float3(1.f, .5f, 1.f));
}
