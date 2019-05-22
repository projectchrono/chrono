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
//
// Contains collision helper functions from ChNarrowphaseR
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"

/// This utility function takes the location 'P' and snaps it to the closest
/// point on the triangular face with given vertices (A, B, and C). The result
/// is returned in 'res'. Both 'P' and 'res' are assumed to be specified in
/// the same frame as the face vertices. This function returns 'true' if the
/// result is on an edge of this face and 'false' if the result is inside the
/// triangle.
/// Code from Ericson, "real-time collision detection", 2005, pp. 141
__device__ bool snap_to_face(const double3& A, const double3& B, const double3& C, const double3& P, double3& res) {
    double3 AB = B - A;
    double3 AC = C - A;

    // Check if P in vertex region outside A
    double3 AP = P - A;
    double d1 = Dot(AB, AP);
    double d2 = Dot(AC, AP);
    if (d1 <= 0 && d2 <= 0) {
        res = A;  // barycentric coordinates (1,0,0)
        return true;
    }

    // Check if P in vertex region outside B
    double3 BP = P - B;
    double d3 = Dot(AB, BP);
    double d4 = Dot(AC, BP);
    if (d3 >= 0 && d4 <= d3) {
        res = B;  // barycentric coordinates (0,1,0)
        return true;
    }

    // Check if P in edge region of AB
    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        // Return projection of P onto AB
        double v = d1 / (d1 - d3);
        res = A + v * AB;  // barycentric coordinates (1-v,v,0)
        return true;
    }

    // Check if P in vertex region outside C
    double3 CP = P - C;
    double d5 = Dot(AB, CP);
    double d6 = Dot(AC, CP);
    if (d6 >= 0 && d5 <= d6) {
        res = C;  // barycentric coordinates (0,0,1)
        return true;
    }

    // Check if P in edge region of AC
    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        // Return projection of P onto AC
        double w = d2 / (d2 - d6);
        res = A + w * AC;  // barycentric coordinates (1-w,0,w)
        return true;
    }

    // Check if P in edge region of BC
    double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        // Return projection of P onto BC
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        res = B + w * (C - B);  // barycentric coordinates (0,1-w,w)
        return true;
    }

    // P inside face region. Return projection of P onto face
    // barycentric coordinates (u,v,w)
    double denom = __drcp_ru(va + vb + vc);
    double v = __dmul_ru(vb, denom);
    double w = __dmul_ru(vc, denom);
    res = A + v * AB + w * AC;  // = u*A + v*B + w*C  where  (u = 1 - v - w)
    return false;
}

__device__ bool face_sphere_cd_inflated(const double3& A,           //!< First vertex of the triangle
                                        const double3& B,           //!< Second vertex of the triangle
                                        const double3& C,           //!< Third vertex of the triangle
                                        const double3& sphere_pos,  //!< Location of the center of the sphere
                                        const int radius,           //!< Sphere radius
                                        const float inflation_radius,
                                        float3& normal,
                                        float& depth,
                                        double3& pt1) {
    // Calculate face normal
    double3 face_n = face_normal(A, B, C);

    // Calculate signed height of sphere center above face plane
    float h = Dot(sphere_pos - A, face_n);

    if (h >= radius || h <= -radius) {
        return false;
    }

    // Find the closest point on the face to the sphere center and determine
    // whether or not this location is inside the face or on an edge.
    double3 faceLoc;

    // Collision detection between sphere and an sphere at the nearest point on the triangle
    snap_to_face(A, B, C, sphere_pos, faceLoc);  // Get faceLoc
    {
        double3 normal_d = sphere_pos - faceLoc;
        normal = make_float3(normal_d.x, normal_d.y, normal_d.z);
    }
    float dist = Length(normal);
    depth = dist - radius - inflation_radius;
    if (depth > 0) {
        return false;
    }
    normal = (1.f / dist) * normal;
    pt1 = faceLoc;  // TODO
    return true;
}

__device__ bool face_sphere_cd_regular(const double3& A,           //!< First vertex of the triangle
                                       const double3& B,           //!< Second vertex of the triangle
                                       const double3& C,           //!< Third vertex of the triangle
                                       const double3& sphere_pos,  //!< Location of the center of the sphere
                                       const int radius,           //!< Sphere radius
                                       float3& normal,
                                       float& depth,
                                       double3& pt1) {
    // Calculate face normal using RHR
    double3 face_n = face_normal(A, B, C);

    // Calculate signed height of sphere center above face plane
    float h = Dot(sphere_pos - A, face_n);

    if (h >= radius || h <= -radius) {
        return false;
    }

    // Find the closest point on the face to the sphere center and determine
    // whether or not this location is inside the face or on an edge.
    double3 faceLoc;

    if (!snap_to_face(A, B, C, sphere_pos, faceLoc)) {
        // Nearest point on the triangle is on its face
        // printf("FACE CONTACT\n");
        depth = h - radius;
        normal = make_float3(face_n.x, face_n.y, face_n.z);
        pt1 = faceLoc;
        return true;  // We are guarenteed contact by an earlier check of h
    } else {
        // printf("EDGE CONTACT\n");
        // Nearest point on the triangle is on an edge
        {
            double3 normal_d = sphere_pos - faceLoc;
            normal = make_float3(normal_d.x, normal_d.y, normal_d.z);
        }
        // normal = make_float3(face_n.x, face_n.y, face_n.z);
        // float dist = Length(sphere_pos - faceLoc);
        float dist = Length(normal);
        depth = dist - radius;
        if (depth >= 0) {
            return false;
        }
        normal = (1.f / dist) * normal;
        pt1 = faceLoc;
        return true;
    }
}

/**
/brief TRIANGLE FACE - SPHERE NARROW-PHASE COLLISION DETECTION

The triangular face is defined by points A, B, C. The sequence is important as it defines the positive face via a
right-hand rule.
The sphere is centered at sphere_pos and has radius.
The index "1" is associated with the triangle. The index "2" is associated with the sphere.
The coordinates of the face and sphere are assumed to be provided in the same reference frame.

Output:
  - pt1:      contact point on triangle
  - depth:    penetration distance (a negative value means that overlap exists)
  - normal:     contact normal, from pt2 to pt1
A return value of "true" signals collision.
*/
__device__ bool face_sphere_cd(const double3& A,           //!< First vertex of the triangle
                               const double3& B,           //!< Second vertex of the triangle
                               const double3& C,           //!< Third vertex of the triangle
                               const double3& sphere_pos,  //!< Location of the center of the sphere
                               const int radius,           //!< Sphere radius
                               const bool inflated,
                               const float inflation_radius,
                               float3& normal,
                               float& depth,
                               double3& pt1) {
    if (inflated) {
        return face_sphere_cd_inflated(A, B, C, sphere_pos, radius, inflation_radius, normal, depth, pt1);
    } else {
        return face_sphere_cd_regular(A, B, C, sphere_pos, radius, normal, depth, pt1);
    }
}