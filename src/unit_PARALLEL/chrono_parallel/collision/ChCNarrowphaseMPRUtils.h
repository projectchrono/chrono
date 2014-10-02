#ifndef CHC_NARROWPHASE_MPR_UTILS_H
#define CHC_NARROWPHASE_MPR_UTILS_H

#include "chrono_parallel/collision/ChCNarrowphase.h"
#include "collision/ChCCollisionModel.h"

inline real3 GetSupportPoint_Sphere(const real3 &B,
                                    const real3 &n) {
   real3 b = real3(B.x);
   return b * b * n / length(b * n);
   //the ellipsoid support function provides a cleaner solution for some reason
   //return B.x * n;
}
inline real3 GetSupportPoint_Triangle(const real3 &A,
                                      const real3 &B,
                                      const real3 &C,
                                      const real3 &n) {
   real dist = dot(A, n);
   real3 point = A;

   if (dot(B, n) > dist) {
      dist = dot(B, n);
      point = B;
   }

   if (dot(C, n) > dist) {
      dist = dot(C, n);
      point = C;
   }

   return point;
}
inline real3 GetSupportPoint_Box(const real3 &B,
                                 const real3 &n) {
   real3 result = R3(0, 0, 0);
   result.x = sign(n.x) * B.x;
   result.y = sign(n.y) * B.y;
   result.z = sign(n.z) * B.z;
   return result;
}
inline real3 GetSupportPoint_Ellipsoid(const real3 &B,
                                       const real3 &n) {

   real3 normal = n;
   real3 result = B * B * normal / length(B * normal);
   //cout << result.x << " " << result.y << " " << result.z<<endl;
   return result;

//
//	real3 norm=normalize(n);
//	real3 dim=(norm*norm)/(B*B);
//	real k = sqrt(1/(dim.x+dim.y+dim.z));
//	return k*norm;
}

inline real3 GetSupportPoint_Cylinder(const real3 &B,
                                      const real3 &n) {
//   real3 u = R3(0, 1, 0);
//   real3 w = n - (dot(u, n)) * u;
//   real3 result;
//
//   if (length(w) != 0) {
//      result = sign(dot(u, n)) * B.y * u + B.x * normalize(w);
//   } else {
//      result = sign(dot(u, n)) * B.y * u;
//   }
//return result;
   real s = sqrt(n.x * n.x + n.z * n.z);
   real3 tmp;
   if (s != 0) {
      tmp.x = n.x * B.x / s;
      tmp.y = n.y < 0.0 ? -B.y : B.y;
      tmp.z = n.z * B.z / s;
   } else {
      tmp.x = B.x;
      tmp.y = n.y < 0.0 ? -B.y : B.y;
      tmp.z = 0;

   }
   return tmp;

}
inline real3 GetSupportPoint_Plane(const real3 &B,
                                   const real3 &n) {
   real3 result = B;

   if (n.x < 0)
      result.x = -result.x;

   if (n.y < 0)
      result.y = -result.y;

   return result;
}
inline real3 GetSupportPoint_Cone(const real3 &B,
                                  const real3 &n) {
   real radius = B.x;
   real height = B.y;

   real m_sinAngle = (radius / sqrt(radius * radius + height * height));

   if (n.y > length(n) * m_sinAngle) {
      return R3(0, height / 2.0, 0);
   } else {
      real s = sqrt(n.x * n.x + n.z * n.z);
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
inline real3 GetSupportPoint_Seg(const real3 &B,
                                 const real3 &n) {
   real3 result = R3(0, 0, 0);
   result.x = sign(n.x) * B.x;

   return result;
}
inline real3 GetSupportPoint_Capsule(const real3 &B,
                                     const real3 &n) {
   return GetSupportPoint_Seg(B, n) + GetSupportPoint_Sphere(R3(B.y), n);

}

inline real3 GetSupportPoint_Disk(const real3 &B,
                                  const real3 &n) {
   real3 n2 = R3(n.x, n.y, 0);
   n2 = normalize(n2);

   real3 result = B.x * n2;

   return result;

}

inline real3 GetSupportPoint_Rect(const real3 &B,
                                  const real3 &n) {
   real3 result = R3(0, 0, 0);
   result.x = sign(n.x) * B.x;
   result.z = sign(n.z) * B.z;
   return result;

}

inline real3 GetSupportPoint_RoundedBox(const real3 &B,
                                        const real3 &C,
                                        const real3 &n) {

   return GetSupportPoint_Box(B, n) + GetSupportPoint_Sphere(R3(C.x), n);

}
inline real3 GetSupportPoint_RoundedCylinder(const real3 &B,
                                             const real3 &C,
                                             const real3 &n) {

   return GetSupportPoint_Cylinder(B, n) + GetSupportPoint_Sphere(R3(C.x), n);

}
inline real3 GetSupportPoint_RoundedCone(const real3 &B,
                                         const real3 &C,
                                         const real3 &n) {

   return GetSupportPoint_Cone(B, n) + GetSupportPoint_Sphere(R3(C.x), n);

}
inline real3 GetCenter_Sphere() {
   return ZERO_VECTOR;
}
inline real3 GetCenter_Triangle(const real3 &A,
                                const real3 &B,
                                const real3 &C) {
   return R3((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f);
}
inline real3 GetCenter_Box() {
   return ZERO_VECTOR;
}
inline real3 GetCenter_Ellipsoid() {
   return ZERO_VECTOR;
}
inline real3 GetCenter_Cylinder() {
   return ZERO_VECTOR;
}
inline real3 GetCenter_Plane() {
   return ZERO_VECTOR;
}
inline real3 GetCenter_Cone(const real3 &B) {
   return ZERO_VECTOR;
}

inline real3 TransformSupportVert(const chrono::collision::ConvexShape &Shape,
                           const real3 &n) {
   real3 localSupport;
   real3 rotated_n = quatRotateMatT(n, Shape.R);
   switch (Shape.type) {
      case chrono::collision::SPHERE:
         localSupport = GetSupportPoint_Sphere(Shape.B, rotated_n);
         break;
      case chrono::collision::ELLIPSOID:
         localSupport = GetSupportPoint_Ellipsoid(Shape.B, rotated_n);
         break;
      case chrono::collision::BOX:
         localSupport = GetSupportPoint_Box(Shape.B, rotated_n);
         break;
      case chrono::collision::CYLINDER:
         localSupport = GetSupportPoint_Cylinder(Shape.B, rotated_n);
         break;
      case chrono::collision::CONE:
         localSupport = GetSupportPoint_Cone(Shape.B, rotated_n);
         break;
      case chrono::collision::CAPSULE:
         localSupport = GetSupportPoint_Capsule(Shape.B, rotated_n);
         break;
      case chrono::collision::ROUNDEDBOX:
         localSupport = GetSupportPoint_RoundedBox(Shape.B, Shape.C, rotated_n);
         break;
      case chrono::collision::ROUNDEDCYL:
         localSupport = GetSupportPoint_RoundedCylinder(Shape.B, Shape.C, rotated_n);
         break;
      case chrono::collision::ROUNDEDCONE:
         localSupport = GetSupportPoint_RoundedCone(Shape.B, Shape.C, rotated_n);
         break;
      case chrono::collision::TRIANGLEMESH:
         return GetSupportPoint_Triangle(Shape.A, Shape.B, Shape.C, n);
         break;
   }

   return quatRotateMat(localSupport, Shape.R) + Shape.A;     //globalSupport
};

#endif
