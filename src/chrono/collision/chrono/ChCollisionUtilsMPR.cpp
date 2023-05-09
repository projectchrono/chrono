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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: Utility functions for GJK and MPR narrowphase.
//
// =============================================================================

#include "chrono/collision/chrono/ChCollisionUtils.h"
#include "chrono/collision/ChCollisionModel.h"

namespace chrono {
namespace collision {
namespace ch_utils {

real3 SupportVertNoMargin(const chrono::collision::ConvexBase* Shape, const real3& nv, const real& envelope) {
    real3 localSupport;
    real3 n = Normalize(nv);
    switch (Shape->Type()) {
        case ChCollisionShape::Type::SPHERE:
            localSupport = GetSupportPoint_Sphere(Shape->Radius(), n);
            break;
        case ChCollisionShape::Type::ELLIPSOID:
            localSupport = GetSupportPoint_Ellipsoid(Shape->Box(), n);
            break;
        case ChCollisionShape::Type::BOX:
            localSupport = GetSupportPoint_Box(Shape->Box(), n);
            break;
        case ChCollisionShape::Type::CYLINDER:
            localSupport = GetSupportPoint_Cylinder(Shape->Box(), n);
            break;
        case ChCollisionShape::Type::CONE:
            localSupport = GetSupportPoint_Cone(Shape->Box(), n);
            break;
        case ChCollisionShape::Type::CAPSULE:
            localSupport = GetSupportPoint_Capsule(Shape->Capsule(), n);
            break;
        case ChCollisionShape::Type::ROUNDEDBOX:
            localSupport = GetSupportPoint_RoundedBox(Shape->Rbox(), n);
            break;
        case ChCollisionShape::Type::ROUNDEDCYL:
            localSupport = GetSupportPoint_RoundedCylinder(Shape->Rbox(), n);
            break;
        case ChCollisionShape::Type::CYLSHELL:
            localSupport = GetSupportPoint_CylindricalShell(Shape->Box(), n);
            break;
        case ChCollisionShape::Type::CONVEX:
            localSupport = GetSupportPoint_Convex(Shape->Size(), Shape->Convex(), n);
            break;
        case ChCollisionShape::Type::TETRAHEDRON:
            localSupport = GetSupportPoint_Tetrahedron(Shape->TetIndex(), Shape->TetNodes(), n);
            break;
    }
    // The collision envelope is applied as a compound support.
    // A sphere with a radius equal to the collision envelope is swept around the
    // entire Shape->
    localSupport += GetSupportPoint_Sphere(envelope, n);
    return localSupport;
}

real3 LocalSupportVert(const chrono::collision::ConvexBase* Shape, const real3& n, const real& envelope) {
    real3 rotated_n = RotateT(n, Shape->R());
    return SupportVertNoMargin(Shape, rotated_n, envelope);
}

real3 TransformSupportVert(const chrono::collision::ConvexBase* Shape, const real3& n, const real& envelope) {
    real3 localSupport;

    switch (Shape->Type()) {
        case ChCollisionShape::Type::TRIANGLE: {
            // Triangles are handled differently than other convex shapes but they
            // still have an envelope around them. Prior to this there was no way
            // to define envelopes for triangle meshes.
            // Note that even with this change, large penetrations might cause the
            // object to be pushed into the triangle mesh. If you need true
            // inside/outside handling please use a convex decomposition
            return GetSupportPoint_Triangle(Shape->Triangles(), n) + GetSupportPoint_Sphere(envelope, n);
            break;
        }
        default:
            localSupport = LocalSupportVert(Shape, n, envelope);
            break;
    }

    return TransformLocalToParent(Shape->A(), Shape->R(), localSupport);
}

// Similar to snap_to_triangle, also returns barycentric coordinates
bool snap_to_triangle_bary(const real3& A,
                           const real3& B,
                           const real3& C,
                           const real3& P,
                           real3& res,
                           real3& barycentric) {
    real3 AB = B - A;
    real3 AC = C - A;

    // Check if P in vertex region outside A
    real3 AP = P - A;
    real d1 = Dot(AB, AP);
    real d2 = Dot(AC, AP);
    if (d1 <= 0 && d2 <= 0) {
        res = A;  // barycentric coordinates (1,0,0)
        barycentric = real3(1.0, 0, 0);
        return true;
    }

    // Check if P in vertex region outside B
    real3 BP = P - B;
    real d3 = Dot(AB, BP);
    real d4 = Dot(AC, BP);
    if (d3 >= 0 && d4 <= d3) {
        res = B;  // barycentric coordinates (0,1,0)
        barycentric = real3(0, 1.0, 0);
        return true;
    }

    // Check if P in edge region of AB
    real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        // Return projection of P onto AB
        real v = d1 / (d1 - d3);
        res = A + v * AB;  // barycentric coordinates (1-v,v,0)
        barycentric = real3(1.0 - v, v, 0);
        return true;
    }

    // Check if P in vertex region outside C
    real3 CP = P - C;
    real d5 = Dot(AB, CP);
    real d6 = Dot(AC, CP);
    if (d6 >= 0 && d5 <= d6) {
        res = C;  // barycentric coordinates (0,0,1)
        barycentric = real3(0, 0, 1.0);
        return true;
    }

    // Check if P in edge region of AC
    real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        // Return projection of P onto AC
        real w = d2 / (d2 - d6);
        res = A + w * AC;  // barycentric coordinates (1-w,0,w)
        barycentric = real3(1.0 - w, 0, w);
        return true;
    }

    // Check if P in edge region of BC
    real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        // Return projection of P onto BC
        real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        res = B + w * (C - B);  // barycentric coordinates (0,1-w,w)
        barycentric = real3(0, 1.0 - w, w);
        return true;
    }

    // P inside face region. Return projection of P onto face
    // barycentric coordinates (u,v,w)
    real denom = 1 / (va + vb + vc);
    real v = vb * denom;
    real w = vc * denom;
    res = A + v * AB + w * AC;  // = u*A + v*B + w*C  where  (u = 1 - v - w)
    barycentric = real3(v, w, 1.0 - v - w);
    return false;
}

// Given a contact point P and a tetrahedron T compute the closest triangle to that point
void FindTriIndex(const real3& P, const uvec4& T, const real3* pos_node, int& face, real3& cb) {
    int i = T.x;
    int j = T.y;
    int k = T.z;
    int l = T.w;

    real3 res, b;
    real bestSqDist = C_REAL_MAX;

    snap_to_triangle_bary(pos_node[j], pos_node[k], pos_node[l], P, res, b);
    real sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 0;
    }
    snap_to_triangle_bary(pos_node[i], pos_node[k], pos_node[l], P, res, b);
    sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 1;
    }
    snap_to_triangle_bary(pos_node[i], pos_node[j], pos_node[l], P, res, b);
    sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 2;
    }
    snap_to_triangle_bary(pos_node[i], pos_node[j], pos_node[k], P, res, b);
    sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 3;
    }
}

}  // end namespace ch_utils
}  // end namespace collision
}  // end namespace chrono
