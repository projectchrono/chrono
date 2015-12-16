#pragma once

#include "chrono_parallel/collision/ChCDataStructures.h"
#include "collision/ChCCollisionModel.h"
namespace chrono {
namespace collision {
inline real3 GetSupportPoint_Sphere(const real& radius, const real3& n) {
    // real3 b = real3(B.x);
    // return b * b * n / length(b * n);
    // the ellipsoid support function provides a cleaner solution for some reason
    return radius * n;
}
inline real3 GetSupportPoint_Triangle(const real3& A, const real3& B, const real3& C, const real3& n) {
    real dist = Dot(A, n);
    real3 point = A;

    if (Dot(B, n) > dist) {
        dist = Dot(B, n);
        point = B;
    }

    if (Dot(C, n) > dist) {
        dist = Dot(C, n);
        point = C;
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
    //   real3 u = R3(0, 1, 0);
    //   real3 w = n - (Dot(u, n)) * u;
    //   real3 result;
    //
    //   if (length(w) != 0) {
    //      result = sign(Dot(u, n)) * B.y * u + B.x * normalize(w);
    //   } else {
    //      result = sign(Dot(u, n)) * B.y * u;
    //   }
    // return result;
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
inline real3 GetSupportPoint_Seg(const real3& B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B.x;

    return result;
}
inline real3 GetSupportPoint_Capsule(const real3& B, const real3& n) {
    return GetSupportPoint_Seg(B, n) + GetSupportPoint_Sphere(B.y, n);
}

inline real3 GetSupportPoint_Disk(const real3& B, const real3& n) {
    real3 n2 = real3(n.x, n.y, 0);
    n2 = Normalize(n2);

    real3 result = B.x * n2;

    return result;
}

inline real3 GetSupportPoint_Rect(const real3& B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B.x;
    result.z = Sign(n.z) * B.z;
    return result;
}

inline real3 GetSupportPoint_RoundedBox(const real3& B, const real3& C, const real3& n) {
    return GetSupportPoint_Box(B, n) + GetSupportPoint_Sphere(C.x, n);
}
inline real3 GetSupportPoint_RoundedCylinder(const real3& B, const real3& C, const real3& n) {
    return GetSupportPoint_Cylinder(B, n) + GetSupportPoint_Sphere(C.x, n);
}
inline real3 GetSupportPoint_RoundedCone(const real3& B, const real3& C, const real3& n) {
    return GetSupportPoint_Cone(B, n) + GetSupportPoint_Sphere(C.x, n);
}

inline real3 GetSupportPoint_Convex(const real3& B, const real3* convex_data, const real3& n) {
    real max_dot_p = -C_LARGE_REAL;
    real dot_p;
    real3 point = convex_data[int(B.y)];
    for (int i = B.y; i < B.y + B.x; i++) {
        dot_p = Dot(convex_data[i], n);
        if (dot_p > max_dot_p) {
            max_dot_p = dot_p;
            point = convex_data[i];
        }
    }
    return point + n * B.z;
}

inline real3 GetCenter_Sphere() {
    return real3(0);
}
inline real3 GetCenter_Triangle(const real3& A, const real3& B, const real3& C) {
    return real3((A.x + B.x + C.x) / real(3.0), (A.y + B.y + C.y) / real(3.0), (A.z + B.z + C.z) / real(3.0));
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
inline real3 GetCenter_Convex(const real3& B, const real3* convex_data) {
    real3 point(0);
    for (int i = B.y; i < B.y + B.x; i++) {
        point += convex_data[i];
    }
    return point / B.x;
}
inline real3 SupportVertNoMargin(const chrono::collision::ConvexShape& Shape, const real3& nv, const real& envelope) {
    real3 localSupport;
    real3 n = Normalize(nv);
    switch (Shape.type) {
        case chrono::collision::SPHERE:
            localSupport = GetSupportPoint_Sphere(Shape.B.x, n);
            break;
        case chrono::collision::ELLIPSOID:
            localSupport = GetSupportPoint_Ellipsoid(Shape.B, n);
            break;
        case chrono::collision::BOX:
            localSupport = GetSupportPoint_Box(Shape.B, n);
            break;
        case chrono::collision::CYLINDER:
            localSupport = GetSupportPoint_Cylinder(Shape.B, n);
            break;
        case chrono::collision::CONE:
            localSupport = GetSupportPoint_Cone(Shape.B, n);
            break;
        case chrono::collision::CAPSULE:
            localSupport = GetSupportPoint_Capsule(Shape.B, n);
            break;
        case chrono::collision::ROUNDEDBOX:
            localSupport = GetSupportPoint_RoundedBox(Shape.B, Shape.C, n);
            break;
        case chrono::collision::ROUNDEDCYL:
            localSupport = GetSupportPoint_RoundedCylinder(Shape.B, Shape.C, n);
            break;
        case chrono::collision::ROUNDEDCONE:
            localSupport = GetSupportPoint_RoundedCone(Shape.B, Shape.C, n);
            break;
        case chrono::collision::CONVEX:
            localSupport = GetSupportPoint_Convex(Shape.B, Shape.convex, n);
            break;
    }
    // The collision envelope is applied as a compound support.
    // A sphere with a radius equal to the collision envelope is swept around the
    // entire shape.
    localSupport += GetSupportPoint_Sphere(envelope, n);
    return localSupport;
}

inline real3 SupportVert(const chrono::collision::ConvexShape& Shape,
                         const real3& nv,
                         const real& envelope,
                         const real& margin = 0) {
    real3 localSupport;
    real3 n = Normalize(nv);
    switch (Shape.type) {
        case chrono::collision::SPHERE:
            localSupport = GetSupportPoint_Sphere(Shape.B.x - margin, n);
            break;
        case chrono::collision::ELLIPSOID:
            localSupport = GetSupportPoint_Ellipsoid(Shape.B - margin, n);
            break;
        case chrono::collision::BOX:
            localSupport = GetSupportPoint_Box(Shape.B - margin, n);
            break;
        case chrono::collision::CYLINDER:
            localSupport = GetSupportPoint_Cylinder(Shape.B - margin, n);
            break;
        case chrono::collision::CONE:
            localSupport = GetSupportPoint_Cone(Shape.B - margin, n);
            break;
        case chrono::collision::CAPSULE:
            localSupport = GetSupportPoint_Capsule(Shape.B - margin, n);
            break;
        case chrono::collision::ROUNDEDBOX:
            localSupport = GetSupportPoint_RoundedBox(Shape.B - margin, Shape.C, n);
            break;
        case chrono::collision::ROUNDEDCYL:
            localSupport = GetSupportPoint_RoundedCylinder(Shape.B - margin, Shape.C, n);
            break;
        case chrono::collision::ROUNDEDCONE:
            localSupport = GetSupportPoint_RoundedCone(Shape.B - margin, Shape.C, n);
            break;
        case chrono::collision::CONVEX:
            localSupport = GetSupportPoint_Convex(Shape.B, Shape.convex, n) - margin * n;
            break;
    }
    // The collision envelope is applied as a compound support.
    // A sphere with a radius equal to the collision envelope is swept around the
    // entire shape.
    localSupport += GetSupportPoint_Sphere(envelope, n) + margin * n;
    return localSupport;
}

inline real3 LocalSupportVert(const chrono::collision::ConvexShape& Shape, const real3& n, const real& envelope) {
    real3 rotated_n = RotateT(n, Shape.R);
    return SupportVertNoMargin(Shape, rotated_n, envelope);
}

inline real3 TransformSupportVert(const chrono::collision::ConvexShape& Shape, const real3& n, const real& envelope) {
    real3 localSupport;

    switch (Shape.type) {
        case chrono::collision::TRIANGLEMESH: {
            // Triangles are handled differently than other convex shapes but they
            // still have an envelope around them. Prior to this there was no way
            // to define envelopes for triangle meshes.
            // Note that even with this change, large penetrations might cause the
            // object to be pushed into the triangle mesh. If you need true
            // inside/outside handling please use a convex decomposition
            return GetSupportPoint_Triangle(Shape.A, Shape.B, Shape.C, n) + GetSupportPoint_Sphere(envelope, n);
            break;
        }
        default:
            localSupport = LocalSupportVert(Shape, n, envelope);
            break;
    }

    return TransformLocalToParent(Shape.A, Shape.R, localSupport);
}

inline void GetBoundingSphere(const chrono::collision::ConvexShape& Shape, real3& center, real& disc) {
    center = real3(0);
    switch (Shape.type) {
        case chrono::collision::SPHERE:
            disc = Shape.B.x;
            break;
        case chrono::collision::ELLIPSOID:
            disc = Max(Shape.B);
            break;
        case chrono::collision::BOX:
            disc = Length(Shape.B);
            break;
        case chrono::collision::CYLINDER:
            disc = Sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y);
            break;
        case chrono::collision::CONE:
            disc = Sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y);
            break;
        case chrono::collision::CAPSULE:
            disc = Shape.B.x + Shape.B.y;
            break;
        case chrono::collision::ROUNDEDBOX:
            disc = Length(Shape.B) + Shape.C.x;
            break;
        case chrono::collision::ROUNDEDCYL:
            disc = Sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y) + Shape.C.x;
            break;
        case chrono::collision::ROUNDEDCONE:
            disc = Sqrt(Shape.B.x * Shape.B.x + Shape.B.y * Shape.B.y) + Shape.C.x;
            break;
    }
}

inline real GetAngularMotionDisc(const chrono::collision::ConvexShape& Shape) {
    real3 center;
    real disc;
    GetBoundingSphere(Shape, center, disc);
    disc += Length(center);
    return disc;
}
}
}
