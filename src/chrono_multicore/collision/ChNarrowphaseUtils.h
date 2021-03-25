// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Utility functions for GJK and MPR narrowphase.
//
// =============================================================================

#pragma once

#include "chrono/collision/ChCollisionModel.h"
#include "chrono_multicore/collision/ChDataStructures.h"

namespace chrono {
namespace collision {

/// @addtogroup multicore_collision
/// @{

/// Support point for a sphere (for GJK and MPR).
inline real3 GetSupportPoint_Sphere(const real& radius, const real3& n) {
    // real3 b = real3(B.x);
    // return b * b * n / length(b * n);
    // the ellipsoid support function provides a cleaner solution for some reason
    return radius * n;
}

/// Support point for a triangle (for GJK and MPR).
inline real3 GetSupportPoint_Triangle(const real3* t, const real3& n) {
    real dist = Dot(t[0], n);
    real3 point = t[0];

    if (Dot(t[1], n) > dist) {
        dist = Dot(t[1], n);
        point = t[1];
    }

    if (Dot(t[2], n) > dist) {
        dist = Dot(t[2], n);
        point = t[2];
    }

    return point;
}

/// Support point for a box (for GJK and MPR).
inline real3 GetSupportPoint_Box(const real3& B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B.x;
    result.y = Sign(n.y) * B.y;
    result.z = Sign(n.z) * B.z;
    return result;
}

/// Support point for an ellipsoid (for GJK and MPR).
inline real3 GetSupportPoint_Ellipsoid(const real3& B, const real3& n) {
    real3 normal = n;
    real3 result = B * B * normal / Length(B * normal);
    // cout << result.x << " " << result.y << " " << result.z<<endl;
    return result;

    //
    //	real3 norm=normalize(n);
    //	real3 dim=(norm*norm)/(B*B);
    //	real k = Sqrt(1/(dim.x+dim.y+dim.z));
    //	return k*norm;
}

/// Support point for a cylinder (for GJK and MPR).
inline real3 GetSupportPoint_Cylinder(const real3& B, const real3& n) {
    real s = Sqrt(n.x * n.x + n.z * n.z);
    real3 tmp;
    if (s != 0) {
        tmp.x = n.x * B.x / s;
        tmp.y = n.y < 0.0 ? -B.y : B.y;
        tmp.z = n.z * B.x / s;
    } else {
        tmp.x = B.x;
        tmp.y = n.y < 0.0 ? -B.y : B.y;
        tmp.z = 0;
    }
    return tmp;
}

/// Support point for a plane (for GJK and MPR).
inline real3 GetSupportPoint_Plane(const real3& B, const real3& n) {
    real3 result = B;

    if (n.x < 0)
        result.x = -result.x;

    if (n.y < 0)
        result.y = -result.y;

    return result;
}

/// Support point for a cone (for GJK and MPR).
inline real3 GetSupportPoint_Cone(const real3& B, const real3& n) {
    real radius = B.x;
    real height = B.y;

    real m_sinAngle = (radius / Sqrt(radius * radius + height * height));

    if (n.y > Length(n) * m_sinAngle) {
        return real3(0, height / 2.0, 0);
    } else {
        real s = Sqrt(n.x * n.x + n.z * n.z);
        if (s > 1e-9) {
            real3 tmp;
            tmp.x = n.x * B.x / s;
            tmp.y = -height / 2.0;
            tmp.z = n.z * B.z / s;
            return tmp;
        } else {
            real3 tmp;
            tmp.x = 0;
            tmp.y = -height / 2.0;
            tmp.z = 0;
            return tmp;
        }
    }
}

/// Support point for a line segment (for GJK and MPR).
inline real3 GetSupportPoint_Seg(const real B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B;

    return result;
}

/// Support point for a capsule (for GJK and MPR).
inline real3 GetSupportPoint_Capsule(const real2& B, const real3& n) {
    return GetSupportPoint_Seg(B.x, n) + GetSupportPoint_Sphere(B.y, n);
}

/// Support point for a disk (for GJK and MPR).
inline real3 GetSupportPoint_Disk(const real& B, const real3& n) {
    real3 n2 = real3(n.x, n.y, 0);
    n2 = Normalize(n2);

    real3 result = B * n2;

    return result;
}

/// Support point for a rectangle (for GJK and MPR).
inline real3 GetSupportPoint_Rect(const real3& B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B.x;
    result.z = Sign(n.z) * B.z;
    return result;
}

/// Support point for a rounded box, i.e. a sphere-swept box (for GJK and MPR).
inline real3 GetSupportPoint_RoundedBox(const real4& B, const real3& n) {
    return GetSupportPoint_Box(real3(B.x, B.y, B.z), n) + GetSupportPoint_Sphere(B.w, n);
}

/// Support point for a rounded cylinder, i.e. a sphere-swept cylinder (for GJK and MPR).
inline real3 GetSupportPoint_RoundedCylinder(const real4& B, const real3& n) {
    return GetSupportPoint_Cylinder(real3(B.x, B.y, B.z), n) + GetSupportPoint_Sphere(B.w, n);
}

/// Support point for a cylindrical shell (for GJK and MPR).
inline real3 GetSupportPoint_CylindricalShell(const real3& B, const real3& n) {
    return GetSupportPoint_Cylinder(real3(B.x, B.y, B.z), n);
}

/// Support point for a rounded cone, i.e. a sphere-swept cone (for GJK and MPR).
inline real3 GetSupportPoint_RoundedCone(const real4& B, const real3& n) {
    return GetSupportPoint_Cone(real3(B.x, B.y, B.z), n) + GetSupportPoint_Sphere(B.w, n);
}

/// Support point for a generic convex shape (for GJK and MPR).
inline real3 GetSupportPoint_Convex(const int size, const real3* convex_data, const real3& n) {
    real max_dot_p = -C_LARGE_REAL;
    real dot_p;
    real3 point = convex_data[0];
    for (int i = 0; i < size; i++) {
        dot_p = Dot(convex_data[i], n);
        if (dot_p > max_dot_p) {
            max_dot_p = dot_p;
            point = convex_data[i];
        }
    }
    return point;
}

/// Support point for a tetrahedron (for GJK and MPR).
inline real3 GetSupportPoint_Tetrahedron(const uvec4 indices, const real3* nodes, const real3& n) {
    real max_dot_p = -C_LARGE_REAL;
    real dot_p;
    real3 point;

    dot_p = Dot(nodes[indices.x], n);
    if (dot_p > max_dot_p) {
        max_dot_p = dot_p;
        point = nodes[indices.x];
    }

    dot_p = Dot(nodes[indices.y], n);
    if (dot_p > max_dot_p) {
        max_dot_p = dot_p;
        point = nodes[indices.y];
    }

    dot_p = Dot(nodes[indices.z], n);
    if (dot_p > max_dot_p) {
        max_dot_p = dot_p;
        point = nodes[indices.z];
    }

    dot_p = Dot(nodes[indices.w], n);
    if (dot_p > max_dot_p) {
        max_dot_p = dot_p;
        point = nodes[indices.w];
    }

    return point;
}

inline real3 GetCenter_Sphere() {
    return real3(0);
}
inline real3 GetCenter_Triangle(const real3* t) {
    return (t[0] + t[1] + t[2]) * 1.0 / 3.0;
}
inline real3 GetCenter_Box() {
    return real3(0);
}
inline real3 GetCenter_Ellipsoid() {
    return real3(0);
}
inline real3 GetCenter_Cylinder() {
    return real3(0);
}
inline real3 GetCenter_Plane() {
    return real3(0);
}
inline real3 GetCenter_Cone(const real3& B) {
    return real3(0);
}
inline real3 GetCenter_Convex(const int size, const real3* convex_data) {
    real3 point(0);
    for (int i = 0; i < size; i++) {
        point += convex_data[i];
    }
    return point / real(size);
}

inline real3 GetCenter_Tetrahedron(const uvec4 indices, const real3* nodes) {
    real3 tet = nodes[indices.x] + nodes[indices.y] + nodes[indices.z] + nodes[indices.w];
    return tet / real(4.0);
}

inline real3 SupportVertNoMargin(const chrono::collision::ConvexBase* Shape, const real3& nv, const real& envelope) {
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
        case ChCollisionShape::Type::ROUNDEDCONE:
            localSupport = GetSupportPoint_RoundedCone(Shape->Rbox(), n);
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

inline real3 LocalSupportVert(const chrono::collision::ConvexBase* Shape, const real3& n, const real& envelope) {
    real3 rotated_n = RotateT(n, Shape->R());
    return SupportVertNoMargin(Shape, rotated_n, envelope);
}

inline real3 TransformSupportVert(const chrono::collision::ConvexBase* Shape, const real3& n, const real& envelope) {
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
// Similar to snap_to_face, also returns barycentric coordinates
static bool SnapeToFaceBary(const real3& A,
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
static void FindTriIndex(const real3& P, const uvec4& T, const real3* pos_node, int& face, real3& cb) {
    int i = T.x;
    int j = T.y;
    int k = T.z;
    int l = T.w;

    real3 res, b;
    real bestSqDist = C_LARGE_REAL;

    SnapeToFaceBary(pos_node[j], pos_node[k], pos_node[l], P, res, b);
    real sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 0;
    }
    SnapeToFaceBary(pos_node[i], pos_node[k], pos_node[l], P, res, b);
    sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 1;
    }
    SnapeToFaceBary(pos_node[i], pos_node[j], pos_node[l], P, res, b);
    sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 2;
    }
    SnapeToFaceBary(pos_node[i], pos_node[j], pos_node[k], P, res, b);
    sqDist = Dot(P - res);
    if (sqDist < bestSqDist) {
        bestSqDist = sqDist;
        cb = b;
        face = 3;
    }
}

/// @} multicore_colision

} // end namespace collision
} // end namespace chrono
