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

#include "chrono/collision/chrono/ChRayTest.h"
#include "chrono/collision/chrono/ChCollisionUtils.h"

// Always include ChConfig.h *before* any Thrust headers!
#include "chrono/ChConfig.h"
#include <thrust/sequence.h>

namespace chrono {
namespace collision {

using namespace chrono::collision::ch_utils;

ChRayTest::ChRayTest(std::shared_ptr<ChCollisionData> data) : cd_data(data) {}

bool ChRayTest::Check(const real3& start, const real3& end, RayHitInfo& info) {
    // Broadphase
    FindCandidates(start, end);
    auto num_candidates = candidate_shapes.size();

    // Narrowphase
    real mindist2 = C_LARGE_REAL;
    ConvexShape shape;
    bool hit = false;
    for (int index = 0; index < (signed)num_candidates; index++) {
        shape.index = candidate_shapes[index];
        shape.data = &cd_data->shape_data;
        if (CheckShape(shape, start, end, info.normal, mindist2))
            hit = true;
    }

    if (hit) {
        real3 ray = end - start;            // Ray vector
        info.shapeID = shape.index;         // Identifier of closest hit shape
        info.dist = Sqrt(mindist2);         // Distance from ray origin
        info.t = info.dist / Length(ray);   // Ray parameter at intersection with closest shape
        info.point = start + info.t * ray;  // Intersection point
    }

    return hit;
}

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
bool aabb_ray(const real3& hdims, const real3& start, const real3& end, real3& loc, real3& normal) {
    real3 ray = end - start;

    bool outside = false;

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

        real t = (locI - start[i]) / ray[i];

        if (t * t > Length2(ray)) {
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
    real3 loc_B;
    real3 normal_B;
    if (aabb_ray(hdims, start_B, end_B, loc_B, normal_B)) {
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
    ////
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
    real ray_len2 = Length2(ray);

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
    real dist2 = t * t * ray_len2;

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
    ////
    return false;
}

// =============================================================================

// Broadphase ray intersection test. It uses results from the collision detection broadphase to exclude shapes from
// possible ray intersection.
void ChRayTest::FindCandidates(const real3& start, const real3& end) {
    //// TODO!!!

    // For now, simply list all existing shapes as candidates
    candidate_shapes.resize(cd_data->num_rigid_shapes);
    thrust::sequence(candidate_shapes.begin(), candidate_shapes.end());
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

}  // end namespace collision
}  // end namespace chrono
