// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/multicore/ChRayTest.h"
#include "chrono/collision/multicore/ChCollisionUtils.h"

// Always include ChConfig.h *before* any Thrust headers!
#include "chrono/ChConfig.h"
#include <thrust/sequence.h>

namespace chrono {

using namespace chrono::ch_utils;

ChRayTest::ChRayTest(std::shared_ptr<ChCollisionData> data) : cd_data(data), num_bin_tests(0), num_shape_tests(0) {}

// =============================================================================

// Test for intersection between a sphere of given `radius` centered at `pos` and the specified oriented line
// segment (from `start` to `end`). An intersection only exists if the square of the distance to the closest
// point of intersection is less than `mindist2`. If an intersection exists, we return the new `mindist2`,
// as well as the `normal` to the object at the intersection point.
bool sphere_ray(const real3& pos,
                const real& radius,
                const real3& start,
                const real3& end,
                real3& normal,
                real& mindist2) {
    real3 vec = start - pos;
    real3 ray = end - start;
    real vv = Length2(vec);
    real a = Length2(ray);
    real b = Dot(vec, ray);
    real c = vv - radius * radius;

    // If the start point is inside or on the sphere, report intersection.
    // Otherwise, no intersection if ray points away from sphere.
    if (c <= 0) {
        if (vv > 0)
            normal = vec / Sqrt(vv);
        else
            normal = real3(0, 0, 1);
        mindist2 = 0;
        return true;
    } else if (b > 0)
        return false;

    // Solve quadratic equation to find ray-sphere intersection: t*t + 2*b*t + c = 0.
    // An intersection exists only if the equation has real solutions and if the smallest
    // one is smaller than both the ray length and the provided 'mindist2'
    real discr = b * b - a * c;
    if (discr < 0)
        return false;

    real t = -(b + Sqrt(discr)) / a;
    if (t >= 1)
        return false;
    real dist2 = a * (t * t);
    if (dist2 > mindist2)
        return false;

    normal = Normalize(vec + t * ray);
    mindist2 = dist2;
    return true;
}

// This function returns true if the specified ray (expressed in the frame of the AABB) passes through the AABB. The
// AABB is assumed to be centered at the origin with no rotation. If the ray intersects the AABB, the location where the
// ray enters the AABB is returned in 'loc' and the normal to the box is returned in 'normal'. If the ray starts inside
// this AABB, the entry location is defined to be the ray's start location and the normal is defined to be the opposite
// of the ray's dir.
bool aabb_ray(const real3& hdims, const real3& start, const real3& end, real& t, real3& loc, real3& normal) {
    real3 ray = end - start;
    bool outside = false;
    t = 0;

    for (int i = 0, j = 1, k = 2; i < 3; j = k, k = i++) {
        real locI;
        real nrmI;

        if (start[i] < -hdims[i]) {
            if (ray[i] <= 0)
                return false;
            nrmI = -1;
            locI = -hdims[i];
        } else if (start[i] > hdims[i]) {
            if (ray[i] >= 0)
                return false;
            nrmI = +1;
            locI = hdims[i];
        } else
            continue;

        t = (locI - start[i]) / ray[i];

        if (t > 1) {
            outside = true;
            continue;
        }

        real locJ = start[j] + ray[j] * t;
        real locK = start[k] + ray[k] * t;

        if (locJ >= -hdims[j] && locJ <= hdims[j] && locK >= -hdims[k] && locK <= hdims[k]) {
            normal[i] = nrmI;
            normal[j] = 0;
            normal[k] = 0;
            loc[i] = locI;
            loc[j] = locJ;
            loc[k] = locK;
            return true;
        }

        outside = true;
    }

    if (!outside) {
        normal = -ray / Length(ray);
        loc = start;
    }

    return !outside;
}

// Test for intersection between a box of given half-dimensions `hdims` centered at `pos` with orientation `rot` and the
// specified oriented line segment (from `start` to `end`). An intersection only exists if the square of the distance to
// the closest point of intersection is less than 'mindist2'. If an intersection exists, we return the new 'mindist2',
// as well as the normal to the object at the intersection point.
bool box_ray(const real3& pos,
             const quaternion& rot,
             const real3& hdims,
             const real3& start,
             const real3& end,
             real3& normal,
             real& mindist2) {
    // Express the ray in the box frame
    real3 start_B = RotateT(start - pos, rot);
    real3 end_B = RotateT(end - pos, rot);

    // Test intersection in local frame (ray against AABB)
    real t_B;
    real3 loc_B;
    real3 normal_B;
    if (aabb_ray(hdims, start_B, end_B, t_B, loc_B, normal_B)) {
        real dist2 = Length2(loc_B - start_B);
        if (dist2 < mindist2) {
            normal = Rotate(normal_B, rot);
            mindist2 = dist2;
            return true;
        }
    }

    return false;
}

// This utility function tests whether the specified line segment (expressed in the frame of the capsule/cylinder)
// intersects the cylindrical portion of a capsule/cylinder. An intersection only exists if the square of the distance
// to the closest point of intersection is less than 'mindist2'. If an intersection exists, we return the new mindist2,
// as well as the normal to the cylinder at the intersection point.
bool cylsurf_ray(const real& radius,
                 const real& hlen,
                 const real3& start,
                 const real3& end,
                 real3& normal,
                 real& mindist2) {
    real3 ray = end - start;
    real ray_len2 = Length2(ray);

    // No intersection if ray (near) parallel to the cylinder's axis (possible intersections of the line segment and the
    // capsule/cylinder in this situation are taken care outside this function).
    if (ray.y * ray.y > 0.9999 * ray_len2)
        return false;

    // The point of intersection between the infinite ray and the cylinder is the solution of a quadratic equation:
    // a*t*t + 2*b*t + c = 0. An intersection exists only if the quadratic equation has real solutions.
    real a = ray.x * ray.x + ray.z * ray.z;
    real b = start.x * ray.x + start.z * ray.z;
    real c = start.x * start.x + start.z * start.z - radius * radius;
    real disc = b * b - a * c;

    if (disc < 0)
        return false;

    real t = -(b + Sqrt(disc)) / a;
    real dist2 = t * t * ray_len2;

    // There is no intersection if the point on the segment lies outside the segment or if it is at a distance larger
    // than 'mindist2'. Also, there is no intersection if the point on the cylinder is outside the bounds.
    if (t < 0 || t > 1 || dist2 > mindist2)
        return false;

    real u = start.y + t * ray.y;

    if (u > hlen || u < -hlen)
        return false;

    normal.x = start.x + t * ray.x;
    normal.y = 0;
    normal.z = start.z + t * ray.z;

    mindist2 = dist2;

    return true;
}

// This utility function tests whether the specified line segment (expressed in the frame of the capsule) intersects the
// endcap sphere located at 'center' (which is either +hlen or -hlen). An intersection only exists if the square of the
// distance to the closest point of intersection is less than 'minDist2'. If an intersection exists, we return the new
// minDist2 as well as the normal to the endcap sphere at the intersection point.
bool hemisphere_ray(int cap,
                    const real& radius,
                    const real& hlen,
                    const real3& start,
                    const real3& end,
                    real3& normal,
                    real& mindist2) {
    //// TODO
    return false;
}

// This utility function tests whether the specified line segment (expressed in the frame of the cylinder) intersects
// the endcap disk located at 'center' (which is either +hlen or -hlen). An intersection only exists if the square of
// the distance to the closest point of intersection is less than 'minDist2'. If an intersection exists, we return the
// new minDist2 as well as the normal to the endcap disk at the intersection point.
bool disk_ray(int cap,
              const real& radius,
              const real& hlen,
              const real3& start,
              const real3& end,
              real3& normal,
              real& mindist2) {
    real3 ray = end - start;

    // No intersection if ray (near) parallel to cylinder cap.
    if (Abs(ray.y) < 0.0001)
        return false;

    // No intersection if start "below" cap
    if (cap * start.y < hlen)
        return false;

    // Normal to cap
    normal.x = 0;
    normal.y = 1.0 * cap;
    normal.z = 0;

    // No intersection if ray points away from cap.
    if (Dot(ray, normal) > 0)
        return false;

    // Intersect infinte ray with cap plane
    real t = (cap * hlen - start.y) / ray.y;
    real dist2 = t * t * Length2(ray);

    if (t < 0 || t > 1 || dist2 > mindist2)
        return false;

    real u = start.x + t * ray.x;
    real v = start.z + t * ray.z;

    if (u * u + v * v > radius * radius)
        return false;

    mindist2 = dist2;
    return true;
}

// Test for intersection between a capsule (centered at 'pos' with orientation given by 'rot' and having given `radius`
// and half-length `hlen`) and the specified oriented line segment (from `start` to `end`). An intersection only exists
// if the square of the distance to the closest point of intersection is less than 'minDist2'. If an intersection
// exists, we return the new 'minDist2', as well as the normal to the object at the intersection point.
bool capsule_ray(const real3& pos,
                 const quaternion& rot,
                 const real& radius,
                 const real& hlen,
                 const real3& start,
                 const real3& end,
                 real3& normal,
                 real& mindist2) {
    // Express the line segment in the frame of the capsule. The capsule itself is centered at the origin and has its
    // direction along the y-axis.
    real3 start_C = RotateT(start - pos, rot);
    real3 end_C = RotateT(end - pos, rot);

    // Separate the capsule into three components, two endcap spheres and a cylindrical segment, and keep track of
    // possible intersection with each one of them. If an intersection exists, express the calculated normal back into
    // the global frame.
    real3 normal_C;
    bool found = cylsurf_ray(radius, hlen, start_C, end_C, normal_C, mindist2);
    found = found || hemisphere_ray(+1, radius, hlen, start_C, end_C, normal_C, mindist2);
    found = found || hemisphere_ray(-1, radius, hlen, start_C, end_C, normal_C, mindist2);

    if (found)
        normal = Rotate(normal_C, rot);

    return found;
}

// Test for intersection between a cylinder (centered at 'pos' with orientation given by 'rot' and having given `radius`
// and half-length `hlen`) and the specified oriented line segment (from `start` to `end`). An intersection only exists
// if the square of the distance to the closest point of intersection is less than 'minDist2'. If an intersection
// exists, we return the new 'minDist2', as well as the normal to the object at the intersection point.
bool cylinder_ray(const real3& pos,
                  const quaternion& rot,
                  const real& radius,
                  const real& hlen,
                  const real3& start,
                  const real3& end,
                  real3& normal,
                  real& mindist2) {
    // Express the line segment in the frame of the cylinder. The cylinder itself is centered at the origin and has its
    // direction along the y-axis.
    real3 start_C = RotateT(start - pos, rot);
    real3 end_C = RotateT(end - pos, rot);

    // Separate the cylinder into three components, two endcap disks and a cylindrical segment, and keep track of
    // possible intersection with each one of them. If an intersection exists, express the calculated normal back into
    // the global frame.
    real3 normal_C;
    bool found = cylsurf_ray(radius, hlen, start_C, end_C, normal_C, mindist2);
    found = found || disk_ray(+1, radius, hlen, start_C, end_C, normal_C, mindist2);
    found = found || disk_ray(-1, radius, hlen, start_C, end_C, normal_C, mindist2);

    if (found)
        normal = Rotate(normal_C, rot);

    return found;
}

bool triangle_ray(const real3& A,
                  const real3& B,
                  const real3& C,
                  const real3& start,
                  const real3& end,
                  real3& normal,
                  real& mindist2) {
    real3 ray = end - start;

    // Calculate face normal.
    normal = triangle_normal(A, B, C);

    // If the 'start' point is below or on the plane of the triangle or 'ray' points away from the triangle, there is no
    // intersection.
    real3 vec = start - A;
    real h = Dot(vec, normal);
    if (h <= 0)
        return false;
    real a = Dot(ray, normal);
    if (a >= 0)
        return false;

    // Find the point of intersection between the infinite ray and the plane of the triangle.  An intersection exists
    // only if this point is inside the triangle and is closer to 'start' than both the segment length and the current
    // 'minDist2' value.
    real t = -h / a;
    real dist2 = t * t * Length2(ray);

    if (dist2 >= mindist2 || t > 1 || !point_in_triangle(A, B, C, start + t * ray))
        return false;

    mindist2 = dist2;
    return true;
}

// =============================================================================

// Use a variant of the 3D Digital Differential Analyser (Akira Fujimoto, "ARTS: Accelerated Ray Tracing Systems", 1986)
// to efficiently traverse the broadphase grid and analytical shape-ray intersection tests.
bool ChRayTest::Check(const real3& start, const real3& end, RayHitInfo& info) {
    // Readability replacements
    const vec3& bins_per_axis = cd_data->bins_per_axis;
    const real3& bin_size = cd_data->bin_size;
    const real3& inv_bin_size = cd_data->inv_bin_size;
    const real3& lbr = cd_data->min_bounding_point;
    const real3& rtf = cd_data->max_bounding_point;
    const std::vector<uint>& bin_start_index_ext = cd_data->bin_start_index_ext;
    const std::vector<uint>& bin_aabb_number = cd_data->bin_aabb_number;

    // Calculate ray parameter at intersection of overall AABB. Return now if no intersection
    real3 center = 0.5 * (rtf + lbr), loc, normal;
    real t_min;
    if (!aabb_ray(0.5 * (rtf - lbr), start - center, end - center, t_min, loc, normal))
        return false;

    // Ray direction
    real3 ray = end - start;

    // Find entry bin
    auto bin = Clamp(HashMin(start - lbr, inv_bin_size), vec3(0, 0, 0), bins_per_axis - vec3(1, 1, 1));

    // Depending on ray sign in each direction:
    // - Initialize next crossing
    // - Set increment in ray parameter at each crossing
    // - Set increment in bin index at each crossing
    // - Set termination criteria (grid exit condition)
    real3 t_next(C_REAL_MAX);
    real3 delta(0);
    vec3 step;
    vec3 exit;
    for (int i = 0; i < 3; i++) {
        real start0 = (start[i] - lbr[i]) + t_min * ray[i];  // ray start point relative to grid LRB
        if (ray[i] < 0) {
            t_next[i] = t_min + (bin[i] * bin_size[i] - start0) / ray[i];
            delta[i] = -bin_size[i] / ray[i];
            step[i] = -1;
            exit[i] = -1;
        }
        if (ray[i] > 0) {
            t_next[i] = t_min + ((bin[i] + 1) * bin_size[i] - start0) / ray[i];
            delta[i] = bin_size[i] / ray[i];
            step[i] = +1;
            exit[i] = bins_per_axis[i];
        }
    }

    // Walk through each bin intersected by the ray (DDA).
    ConvexShape shape(-1, &cd_data->shape_data);
    real mindist2 = C_REAL_MAX;
    bool hit = false;

    ////std::cout << "Ray start: [" << start.x << "," << start.y << "," << start.z << "]" << std::endl;
    ////std::cout << "Ray end:   [" << end.x << "," << end.y << "," << end.z << "]" << std::endl;

    while (true) {
        ////std::cout << "  Test BIN:  [" << bin.x << "," << bin.y << "," << bin.z << "]" << std::endl;
        num_bin_tests++;

        // Test ray against all shapes in current bin.
        auto bin_index = Hash_Index(bin, bins_per_axis);
        auto start_index = bin_start_index_ext[bin_index];
        auto end_index = bin_start_index_ext[bin_index + 1];

        for (uint j = start_index; j < end_index; j++) {
            num_shape_tests++;
            shape.index = bin_aabb_number[j];
            ////std::cout << "    Test SHAPE: " << shape.index << std::endl;
            if (CheckShape(shape, start, end, info.normal, mindist2))
                hit = true;
        }

        // If a shape in the current bin was hit, stop.
        if (hit) {
            info.shapeID = shape.index;         // Identifier of closest hit shape
            info.dist = Sqrt(mindist2);         // Distance from ray origin
            info.t = info.dist / Length(ray);   // Ray parameter at intersection with closest shape
            info.point = start + info.t * ray;  // Intersection point
            break;
        }

        // Move to the next cell (the one with lowest t_next)
        static const int map[8] = {2, 1, 2, 1, 2, 2, 0, 0};
        int k = ((t_next[0] < t_next[1]) << 2) + ((t_next[0] < t_next[2]) << 1) + ((t_next[1] < t_next[2]));
        int axis = map[k];
        bin[axis] += step[axis];
        if (bin[axis] == exit[axis])
            break;
        t_next[axis] += delta[axis];
    }

    return hit;
}

// Narrowphase dispatcher for ray intersection test.  It uses analytical formulaes for known primitive shapes with
// fallback on a generic ray-convex intersection test.
bool ChRayTest::CheckShape(const ConvexBase& shape,
                           const real3& start,
                           const real3& end,
                           real3& normal,
                           real& mindist2) {
    auto shape_type = shape.Type();

    switch (shape_type) {
        case ChCollisionShape::Type::SPHERE:
            return sphere_ray(shape.A(), shape.Radius(), start, end, normal, mindist2);
        case ChCollisionShape::Type::BOX:
            return box_ray(shape.A(), shape.R(), shape.Box(), start, end, normal, mindist2);
        case ChCollisionShape::Type::CYLINDER:
            return cylinder_ray(shape.A(), shape.R(), shape.Box().x, shape.Box().y, start, end, normal, mindist2);
        case ChCollisionShape::Type::CAPSULE:
            return capsule_ray(shape.A(), shape.R(), shape.Capsule().x, shape.Capsule().y, start, end, normal,
                               mindist2);
        case ChCollisionShape::Type::TRIANGLE:
            return triangle_ray(shape.Triangles()[0], shape.Triangles()[1], shape.Triangles()[2], start, end, normal,
                                mindist2);
        default:
            //// TODO: fallback on generic ray-convex intersection test
            return false;
    }
}

}  // end namespace chrono
