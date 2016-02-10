#pragma once

#include "collision/ChCCollisionModel.h"
#include "chrono_parallel/collision/ChDataStructures.h"
namespace chrono {
namespace collision {
inline real3 GetSupportPoint_Sphere(const real& radius, const real3& n) {
    // real3 b = real3(B.x);
    // return b * b * n / length(b * n);
    // the ellipsoid support function provides a cleaner solution for some reason
    return radius * n;
}
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
inline real3 GetSupportPoint_Box(const real3& B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B.x;
    result.y = Sign(n.y) * B.y;
    result.z = Sign(n.z) * B.z;
    return result;
}
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
inline real3 GetSupportPoint_Plane(const real3& B, const real3& n) {
    real3 result = B;

    if (n.x < 0)
        result.x = -result.x;

    if (n.y < 0)
        result.y = -result.y;

    return result;
}
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
inline real3 GetSupportPoint_Seg(const real B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B;

    return result;
}
inline real3 GetSupportPoint_Capsule(const real2& B, const real3& n) {
    return GetSupportPoint_Seg(B.x, n) + GetSupportPoint_Sphere(B.y, n);
}

inline real3 GetSupportPoint_Disk(const real& B, const real3& n) {
    real3 n2 = real3(n.x, n.y, 0);
    n2 = Normalize(n2);

    real3 result = B * n2;

    return result;
}

inline real3 GetSupportPoint_Rect(const real3& B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B.x;
    result.z = Sign(n.z) * B.z;
    return result;
}

inline real3 GetSupportPoint_RoundedBox(const real4& B, const real3& n) {
    return GetSupportPoint_Box(real3(B.x, B.y, B.z), n) + GetSupportPoint_Sphere(B.w, n);
}
inline real3 GetSupportPoint_RoundedCylinder(const real4& B, const real3& n) {
    return GetSupportPoint_Cylinder(real3(B.x, B.y, B.z), n) + GetSupportPoint_Sphere(B.w, n);
}
inline real3 GetSupportPoint_RoundedCone(const real4& B, const real3& n) {
    return GetSupportPoint_Cone(real3(B.x, B.y, B.z), n) + GetSupportPoint_Sphere(B.w, n);
}

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

inline real3 GetSupportPoint_Tetrahedron(const uint4 indices, const real3* nodes, const real3& n) {
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

inline real3 GetCenter_Tetrahedron(const uint4 indices, const real3* nodes) {
    real3 tet = nodes[indices.x] + nodes[indices.y] + nodes[indices.z] + nodes[indices.w];
    return tet / real(4.0);
}

inline real3 SupportVertNoMargin(const chrono::collision::ConvexBase* Shape, const real3& nv, const real& envelope) {
    real3 localSupport;
    real3 n = Normalize(nv);
    switch (Shape->Type()) {
        case chrono::collision::SPHERE:
            localSupport = GetSupportPoint_Sphere(Shape->Radius(), n);
            break;
        case chrono::collision::ELLIPSOID:
            localSupport = GetSupportPoint_Ellipsoid(Shape->Box(), n);
            break;
        case chrono::collision::BOX:
            localSupport = GetSupportPoint_Box(Shape->Box(), n);
            break;
        case chrono::collision::CYLINDER:
            localSupport = GetSupportPoint_Cylinder(Shape->Box(), n);
            break;
        case chrono::collision::CONE:
            localSupport = GetSupportPoint_Cone(Shape->Box(), n);
            break;
        case chrono::collision::CAPSULE:
            localSupport = GetSupportPoint_Capsule(Shape->Capsule(), n);
            break;
        case chrono::collision::ROUNDEDBOX:
            localSupport = GetSupportPoint_RoundedBox(Shape->Rbox(), n);
            break;
        case chrono::collision::ROUNDEDCYL:
            localSupport = GetSupportPoint_RoundedCylinder(Shape->Rbox(), n);
            break;
        case chrono::collision::ROUNDEDCONE:
            localSupport = GetSupportPoint_RoundedCone(Shape->Rbox(), n);
            break;
        case chrono::collision::CONVEX:
            localSupport = GetSupportPoint_Convex(Shape->Size(), Shape->Convex(), n);
            break;
        case chrono::collision::TETRAHEDRON:
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
        case chrono::collision::TRIANGLEMESH: {
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
}
}
