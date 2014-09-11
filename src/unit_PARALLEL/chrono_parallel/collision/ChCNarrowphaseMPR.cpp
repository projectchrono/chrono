#include <algorithm>

#include "collision/ChCCollisionModel.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "ChCNarrowphaseMPR.h"
#include "ChCNarrowphaseMPRUtils.h"

using namespace chrono::collision;

#define MPR_TOLERANCE  0
#define MAX_ITERATIONS  1000
struct support {
   real3 v, v1, v2;

};

struct simplex {
   support s0, s1, s2, s3, s4;
};

bool chrono::collision::SphereSphere(const real3 &A_X,
                                     const real3 &B_X,
                                     const real3 &A_Y,
                                     const real3 &B_Y,
                                     real3 & N,
                                     real & depth,
                                     real3 & p1,
                                     real3 & p2) {
   real3 relpos = B_X - A_X;
   real d2 = dot(relpos, relpos);
   real collide_dist = A_Y.x + B_Y.x;
   if (d2 <= collide_dist * collide_dist) {
      //depth = sqrtf(d2)-collide_dist;
      N = normalize(relpos);
      p1 = A_X + N * A_Y.x;
      p2 = B_X - N * B_Y.x;
      depth = dot(N, p2 - p1);
      return true;
   }
   return false;
}

__device__ __host__ real3 GetCenter(const shape_type &type,
                                    const real3 &A,
                                    const real3 &B,
                                    const real3 &C) {
   if (type == TRIANGLEMESH) {
      return GetCenter_Triangle(A, B, C);     //triangle center
   } else {
      return R3(0, 0, 0) + A;     //All other shapes assumed to be locally centered
   }
}

__device__ __host__ real3 TransformSupportVert(const shape_type &type,
                                               const real3 &A,
                                               const real3 &B,
                                               const real3 &C,
                                               const real4 &R,
                                               const real3 &n) {
   real3 localSupport;
   real3 rotated_n = quatRotateMatT(n, R);
   switch (type) {
      case chrono::collision::SPHERE:
         localSupport = GetSupportPoint_Sphere(B, rotated_n);
         break;
      case chrono::collision::ELLIPSOID:
         localSupport = GetSupportPoint_Ellipsoid(B, rotated_n);
         break;
      case chrono::collision::BOX:
         localSupport = GetSupportPoint_Box(B, rotated_n);
         break;
      case chrono::collision::CYLINDER:
         localSupport = GetSupportPoint_Cylinder(B, rotated_n);
         break;
      case chrono::collision::CONE:
         localSupport = GetSupportPoint_Cone(B, rotated_n);
         break;
      case chrono::collision::CAPSULE:
         localSupport = GetSupportPoint_Capsule(B, rotated_n);
         break;
      case chrono::collision::ROUNDEDBOX:
         localSupport = GetSupportPoint_RoundedBox(B, C, rotated_n);
         break;
      case chrono::collision::ROUNDEDCYL:
         localSupport = GetSupportPoint_RoundedCylinder(B, C, rotated_n);
         break;
      case chrono::collision::ROUNDEDCONE:
         localSupport = GetSupportPoint_RoundedCone(B, C, rotated_n);
         break;
      case chrono::collision::TRIANGLEMESH:
         return GetSupportPoint_Triangle(A, B, C, n);
         break;
   }

   return quatRotateMat(localSupport, R) + A;     //globalSupport
}

void FindCenter(const shape_type &typeA,
                const real3 &A_X,
                const real3 &A_Y,
                const real3 &A_Z,
                const shape_type &typeB,
                const real3 &B_X,
                const real3 &B_Y,
                const real3 &B_Z,
                simplex & portal) {

   // v0 = center of Minkowski sum
   portal.s0.v1 = GetCenter(typeA, A_X, A_Y, A_Z);
   portal.s0.v2 = GetCenter(typeB, B_X, B_Y, B_Z);
   portal.s0.v = portal.s0.v2 - portal.s0.v1;

}

void MPRSupport(const shape_type &typeA,
                const real3 &A_X,
                const real3 &A_Y,
                const real3 &A_Z,
                const real4 &A_R,
                const shape_type &typeB,
                const real3 &B_X,
                const real3 &B_Y,
                const real3 &B_Z,
                const real4 &B_R,
                const real3 & n,
                support & s) {

   s.v1 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
   s.v2 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
   s.v = s.v2 - s.v1;

}

void ExpandPortal(simplex & portal) {

   // Compute the tetrahedron dividing face (v4,v0,v1)
   if (dot(cross(portal.s4.v, portal.s1.v), portal.s0.v) < 0) {
      // Compute the tetrahedron dividing face (v4,v0,v2)
      if (dot(cross(portal.s4.v, portal.s2.v), portal.s0.v) < 0) {
         portal.s1 = portal.s4;  // Inside d1 & inside d2 ==> eliminate v1
      } else {
         portal.s3 = portal.s4;  // Inside d1 & outside d2 ==> eliminate v3
      }
   } else {
      // Compute the tetrahedron dividing face (v4,v0,v3)
      if (dot(cross(portal.s4.v, portal.s3.v), portal.s0.v) < 0) {
         portal.s2 = portal.s4;  // Outside d1 & inside d3 ==> eliminate v2
      } else {
         portal.s1 = portal.s4;  // Outside d1 & outside d3 ==> eliminate v1
      }
   }

}

real3 PortalDir(const simplex & portal) {
   return normalize(cross((portal.s2.v - portal.s1.v), (portal.s3.v - portal.s1.v)));

}

void FindPos(const simplex & portal,
             real3 & point) {
   real3 n = PortalDir(portal);
   // Compute the barycentric coordinates of the origin
   real b0 = dot(cross(portal.s1.v, portal.s2.v), portal.s3.v);
   real b1 = dot(cross(portal.s3.v, portal.s2.v), portal.s0.v);
   real b2 = dot(cross(portal.s0.v, portal.s1.v), portal.s3.v);
   real b3 = dot(cross(portal.s2.v, portal.s1.v), portal.s0.v);
   real sum = b0 + b1 + b2 + b3;

   if (sum <= 0.) {
      b0 = 0;
      b1 = dot(cross(portal.s2.v, portal.s3.v), n);
      b2 = dot(cross(portal.s3.v, portal.s1.v), n);
      b3 = dot(cross(portal.s1.v, portal.s2.v), n);
      sum = b1 + b2 + b3;
   }

   real inv = 1.0f / sum;
   real3 p1 = (b0 * portal.s0.v1 + b1 * portal.s1.v1 + b2 * portal.s2.v1 + b3 * portal.s3.v1) * inv;
   real3 p2 = (b0 * portal.s0.v2 + b1 * portal.s1.v2 + b2 * portal.s2.v2 + b3 * portal.s3.v2) * inv;
   point = (p1 + p2) * 0.5f;

}

int portalEncapsulesOrigin(const simplex & portal,
                           const real3 & n) {

   return dot(n, portal.s1.v) >= 0.0;

}

int portalCanEncapsulesOrigin(const simplex & portal,
                              const real3 &n) {

   return dot(portal.s4.v, n) >= 0.0;

}

int portalReachTolerance(const simplex & portal,
                         const real3 &n) {

   real dv1 = dot(portal.s1.v, n);
   real dv2 = dot(portal.s2.v, n);
   real dv3 = dot(portal.s3.v, n);
   real dv4 = dot(portal.s4.v, n);

   real dot1 = dv4 - dv1;
   real dot2 = dv4 - dv2;
   real dot3 = dv4 - dv3;

   dot1 = std::fmin(dot1, dot2);
   dot1 = std::fmin(dot1, dot3);

   return isEqual(dot1, MPR_TOLERANCE) || dot1 < MPR_TOLERANCE;

}

real Vec3PointSegmentDist2(const real3 & P,
                           const real3 & x0,
                           const real3 & b) {
   real dist, t;
   real3 d, a;
   d = b - x0;     // direction of segment
   a = x0 - P;     // precompute vector from P to x0
   t = -(1.f) * dot(a, d);
   t = t / dot(d, d);

   if (t < 0.0f || IsZero(t)) {
      dist = dot(x0 - P, x0 - P);

   } else if (t > 1.0f || isEqual(t, 1.0f)) {
      dist = dot(b - P, b - P);

   } else {
      d = d * t;
      d = d * a;
      d = dot(d, d);
   }

   return dist;
}
__device__ __host__ real Vec3PointTriDist2(const real3 & P,
                                           const real3 & V0,
                                           const real3 & V1,
                                           const real3 & V2) {
//   real3 d1, d2, a;
//   real u, v, w, p, q, r;
//   real s, t, dist, dist2;
//
//   d1 = B - x0;
//   d2 = C - x0;
//   a = x0 - P;
//
//   u = dot(a, a);
//   v = dot(d1, d1);
//   w = dot(d2, d2);
//   p = dot(a, d1);
//   q = dot(a, d2);
//   r = dot(d1, d2);
//
//   s = (q * r - w * p) / (w * v - r * r);
//   t = (-s * r - q) / w;
//
//   if ((IsZero(s) || s > 0.0f) && (isEqual(s, 1.0f) || s < 1.0f) && (IsZero(t) || t > 0.0f) && (isEqual(t, 1.0f) || t < 1.0f) && (isEqual(t + s, 1.0f) || t + s < 1.0f)) {
//
//      dist = s * s * v;
//      dist += t * t * w;
//      dist += 2.f * s * t * r;
//      dist += 2.f * s * p;
//      dist += 2.f * t * q;
//      dist += u;
//
//   } else {
//
//      dist = Vec3PointSegmentDist2(P, x0, B);
//      dist2 = Vec3PointSegmentDist2(P, x0, C);
//
//      if (dist2 < dist) {
//         dist = dist2;
//      }
//
//      dist2 = Vec3PointSegmentDist2(P, B, C);
//
//      if (dist2 < dist) {
//         dist = dist2;
//
//      }
//   }
//
//   return dist;

   real3 diff = V0 - P;
   real3 edge0 = V1 - V0;
   real3 edge1 = V2 - V0;
   real a00 = dot(edge0, edge0);
   real a01 = dot(edge0, edge1);
   real a11 = dot(edge1, edge1);
   real b0 = dot(diff, edge0);
   real b1 = dot(diff, edge1);
   real c = dot(diff, diff);
   real det = fabs(a00 * a11 - a01 * a01);
   real s = a01 * b1 - a11 * b0;
   real t = a01 * b0 - a00 * b1;
   real sqrDistance;

   if (s + t <= det) {
      if (s < (real) 0) {
         if (t < (real) 0)  // region 4
               {
            if (b0 < (real) 0) {
               t = (real) 0;
               if (-b0 >= a00) {
                  s = (real) 1;
                  sqrDistance = a00 + ((real) 2) * b0 + c;
               } else {
                  s = -b0 / a00;
                  sqrDistance = b0 * s + c;
               }
            } else {
               s = (real) 0;
               if (b1 >= (real) 0) {
                  t = (real) 0;
                  sqrDistance = c;
               } else if (-b1 >= a11) {
                  t = (real) 1;
                  sqrDistance = a11 + ((real) 2) * b1 + c;
               } else {
                  t = -b1 / a11;
                  sqrDistance = b1 * t + c;
               }
            }
         } else  // region 3
         {
            s = (real) 0;
            if (b1 >= (real) 0) {
               t = (real) 0;
               sqrDistance = c;
            } else if (-b1 >= a11) {
               t = (real) 1;
               sqrDistance = a11 + ((real) 2) * b1 + c;
            } else {
               t = -b1 / a11;
               sqrDistance = b1 * t + c;
            }
         }
      } else if (t < (real) 0)  // region 5
            {
         t = (real) 0;
         if (b0 >= (real) 0) {
            s = (real) 0;
            sqrDistance = c;
         } else if (-b0 >= a00) {
            s = (real) 1;
            sqrDistance = a00 + ((real) 2) * b0 + c;
         } else {
            s = -b0 / a00;
            sqrDistance = b0 * s + c;
         }
      } else  // region 0
      {
         // minimum at interior point
         real invDet = ((real) 1) / det;
         s *= invDet;
         t *= invDet;
         sqrDistance = s * (a00 * s + a01 * t + ((real) 2) * b0) + t * (a01 * s + a11 * t + ((real) 2) * b1) + c;
      }
   } else {
      real tmp0, tmp1, numer, denom;

      if (s < (real) 0)  // region 2
            {
         tmp0 = a01 + b0;
         tmp1 = a11 + b1;
         if (tmp1 > tmp0) {
            numer = tmp1 - tmp0;
            denom = a00 - ((real) 2) * a01 + a11;
            if (numer >= denom) {
               s = (real) 1;
               t = (real) 0;
               sqrDistance = a00 + ((real) 2) * b0 + c;
            } else {
               s = numer / denom;
               t = (real) 1 - s;
               sqrDistance = s * (a00 * s + a01 * t + ((real) 2) * b0) + t * (a01 * s + a11 * t + ((real) 2) * b1) + c;
            }
         } else {
            s = (real) 0;
            if (tmp1 <= (real) 0) {
               t = (real) 1;
               sqrDistance = a11 + ((real) 2) * b1 + c;
            } else if (b1 >= (real) 0) {
               t = (real) 0;
               sqrDistance = c;
            } else {
               t = -b1 / a11;
               sqrDistance = b1 * t + c;
            }
         }
      } else if (t < (real) 0)  // region 6
            {
         tmp0 = a01 + b1;
         tmp1 = a00 + b0;
         if (tmp1 > tmp0) {
            numer = tmp1 - tmp0;
            denom = a00 - ((real) 2) * a01 + a11;
            if (numer >= denom) {
               t = (real) 1;
               s = (real) 0;
               sqrDistance = a11 + ((real) 2) * b1 + c;
            } else {
               t = numer / denom;
               s = (real) 1 - t;
               sqrDistance = s * (a00 * s + a01 * t + ((real) 2) * b0) + t * (a01 * s + a11 * t + ((real) 2) * b1) + c;
            }
         } else {
            t = (real) 0;
            if (tmp1 <= (real) 0) {
               s = (real) 1;
               sqrDistance = a00 + ((real) 2) * b0 + c;
            } else if (b0 >= (real) 0) {
               s = (real) 0;
               sqrDistance = c;
            } else {
               s = -b0 / a00;
               sqrDistance = b0 * s + c;
            }
         }
      } else  // region 1
      {
         numer = a11 + b1 - a01 - b0;
         if (numer <= (real) 0) {
            s = (real) 0;
            t = (real) 1;
            sqrDistance = a11 + ((real) 2) * b1 + c;
         } else {
            denom = a00 - ((real) 2) * a01 + a11;
            if (numer >= denom) {
               s = (real) 1;
               t = (real) 0;
               sqrDistance = a00 + ((real) 2) * b0 + c;
            } else {
               s = numer / denom;
               t = (real) 1 - s;
               sqrDistance = s * (a00 * s + a01 * t + ((real) 2) * b0) + t * (a01 * s + a11 * t + ((real) 2) * b1) + c;
            }
         }
      }
   }

   // Account for numerical round-off error.
   if (sqrDistance < (real) 0) {
      sqrDistance = (real) 0;
   }

   //mClosestPoint0 = P;
   //mClosestPoint1 = V0 + s * edge0 + t * edge1;
   //mTriangleBary[1] = s;
   //mTriangleBary[2] = t;
   //mTriangleBary[0] = (real) 1 - s - t;
   return sqrDistance;

}

void FindPenetration(const shape_type & typeA,
                     const real3 & A_X,
                     const real3 & A_Y,
                     const real3 & A_Z,
                     const real4 & A_R,
                     const shape_type & typeB,
                     const real3 & B_X,
                     const real3 & B_Y,
                     const real3 & B_Z,
                     const real4 & B_R,
                     simplex & portal,
                     real & depth,
                     real3 & n,
                     real3 & point) {
   real3 zero = real3(0);

   for (int i = 0; i < MAX_ITERATIONS; i++) {

      n = PortalDir(portal);
      //cout<<"PortalDir"<<n<<endl;
      MPRSupport(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, n, portal.s4);

      real delta = dot((portal.s4.v - portal.s3.v), n);

      if (portalReachTolerance(portal, n) || i == MAX_ITERATIONS) {
         //depth = -sqrtf(Vec3PointTriDist2(zero, portal.s1.v, portal.s2.v, portal.s3.v));
         //if (depth != depth) {
         //   depth = 0;
         //}
         //dir = n;
         FindPos(portal, point);

         return;
      }
      ExpandPortal(portal);
   }

//   real3 n = PortalDir(portal);
//   depth = -sqrtf(Vec3PointTriDist2(zero, portal.s1.v, portal.s2.v, portal.s3.v));
//   if (depth != depth) {
//      depth = 0;
//   }
//   dir = n;
//   FindPos(portal, point);
}

bool FindPortal(const shape_type & typeA,
                const real3 &A_X,
                const real3 &A_Y,
                const real3 &A_Z,
                const real4 &A_R,
                const shape_type &typeB,
                const real3 &B_X,
                const real3 &B_Y,
                const real3 &B_Z,
                const real4 &B_R,
                simplex & portal,
                real3 & n) {

   // Phase One: Identify a portal
   while (true) {

      // Obtain the support point in a direction perpendicular to the existing plane
      // Note: This point is guaranteed to lie off the plane
      MPRSupport(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, n, portal.s3);

      if (dot(portal.s3.v, n) <= 0.0) {
         //cout << "FAIL C" << endl;
         return false;
      }
      // If origin is outside (v1,v0,v3), then eliminate v2 and loop
      if (dot(cross(portal.s1.v, portal.s3.v), portal.s0.v) < 0.0) {
         portal.s2 = portal.s3;
         n = normalize(cross((portal.s1.v - portal.s0.v), (portal.s3.v - portal.s0.v)));
         continue;
      }
      // If origin is outside (v3,v0,v2), then eliminate v1 and loop
      if (dot(cross(portal.s3.v, portal.s2.v), portal.s0.v) < 0.0) {
         portal.s1 = portal.s3;
         n = normalize(cross((portal.s3.v - portal.s0.v), (portal.s2.v - portal.s0.v)));
         continue;
      }
      break;
   }
   return true;

}
//Code for Convex-Convex Collision detection, adopted from xeno-collide
bool chrono::collision::CollideAndFindPoint(const shape_type &typeA,
                                            const real3 &A_X,
                                            const real3 &A_Y,
                                            const real3 &A_Z,
                                            const real4 &A_R,
                                            const shape_type &typeB,
                                            const real3 &B_X,
                                            const real3 &B_Y,
                                            const real3 &B_Z,
                                            const real4 &B_R,
                                            real3 &returnNormal,
                                            real3 &point,
                                            real &depth) {

   real3 n;
   simplex portal;

   FindCenter(typeA, A_X, A_Y, A_Z, typeB, B_X, B_Y, B_Z, portal);

// Avoid case where centers overlap -- any direction is fine in this case
   if (IsZero(portal.s0.v)) {
      portal.s0.v = R3(1, 0, 0);
   }
// v1 = support in direction of origin
   n = normalize(-portal.s0.v);
   MPRSupport(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, n, portal.s1);

   if (dot(portal.s1.v, n) < 0.0) {
      //no contact
      //cout << "FAIL A " << dot(portal.s1.v, n) << endl;
      return false;
   }

// v2 - support perpendicular to v1,v0
   n = cross(portal.s1.v, portal.s0.v);

   if (IsZero(n)) {
      if (IsZero(portal.s1.v)) {
         depth = 0;
         n = portal.s1.v - portal.s0.v;
      } else {
         n = portal.s1.v;
         depth = -length(n);
      }
      point = (portal.s1.v1 + portal.s1.v2) * .5;
      n = normalize(n);
      returnNormal = n;
      //n = portal.s1.v - portal.s0.v;
      //n = normalize(n);
      //cout << "EXIT A" << endl;
      return true;
   }
   n = normalize(n);

   MPRSupport(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, n, portal.s2);
   if (dot(portal.s2.v, n) <= 0.0) {
      //cout << "FAIL B" << endl;
      return false;
   }

// Determine whether origin is on + or - side of plane (v1,v0,v2)
   n = normalize(cross((portal.s1.v - portal.s0.v), (portal.s2.v - portal.s0.v)));
// If the origin is on the - side of the plane, reverse the direction of the plane
   if (dot(n, portal.s0.v) > 0.0) {
      Swap(portal.s1, portal.s2);
      n = -n;
   }
//   cout << n << " " << portal.s0.v << portal.s1.v << portal.s2.v << (portal.s1.v - portal.s0.v) << (portal.s2.v - portal.s0.v)
//        << cross((portal.s1.v - portal.s0.v), (portal.s2.v - portal.s0.v)) << endl;
   if (!FindPortal(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, portal, n)) {
      return false;
   }
   // Phase Two: Refine the portal
   // We are now inside of a wedge...
   bool hit = false;
   for (int i = 0; i < MAX_ITERATIONS; i++) {
      // Compute normal of the wedge face
      n = PortalDir(portal);
      // Find the support point in the direction of the wedge face
      MPRSupport(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, n, portal.s4);

      // If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
      if (portalReachTolerance(portal, n) || !portalEncapsulesOrigin(portal, n)) {
         //FindPortal(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, portal, n);
         // Compute distance from origin to wedge face
         // If the origin is inside the wedge, we have a hit
         if (portalEncapsulesOrigin(portal, n) >= 0.0 && !hit) {
            //cout << "Hit" << n << endl;
            hit = true;     // HIT!!!
         }
         break;
      }
      ExpandPortal(portal);

   }
   if (hit) {
      //depth = dot(portal.s4.v, n);
      FindPenetration(typeA, A_X, A_Y, A_Z, A_R, typeB, B_X, B_Y, B_Z, B_R, portal, depth, returnNormal, point);
      //cout<<returnNormal<<endl;

      //
//      if (depth > 0) {
//         return false;
//      }
      //cout<<A_X<<A_Y<<B_X<<B_Y<<endl;//

      //cout<<portal.s0.v<<portal.s1.v<<portal.s2.v<<portal.s3.v<<portal.s4.v<<endl;
      //n = PortalDir(portal);
      //FindPos(portal, point);
      //returnNormal = normalize(n);
      //exit(0);
   }
   return hit;

}
void chrono::collision::GetPoints(
                                  shape_type A_T,
                                  real3 A_X,
                                  real3 A_Y,
                                  real3 A_Z,
                                  real4 A_R,
                                  shape_type B_T,
                                  real3 B_X,
                                  real3 B_Y,
                                  real3 B_Z,
                                  real4 B_R,
                                  real3 &N,
                                  real3 p0,
                                  real3 & p1,
                                  real3 & p2) {

   p1 = dot((TransformSupportVert(A_T, A_X, A_Y, A_Z, A_R, -N) - p0), N) * N + p0;
   p2 = dot((TransformSupportVert(B_T, B_X, B_Y, B_Z, B_R, N) - p0), N) * N + p0;
   N = -N;

}
void ChCNarrowphaseMPR::function_MPR_Store(const uint &index,
                                           const shape_type *obj_data_T,
                                           const real3 *obj_data_A,
                                           const real3 *obj_data_B,
                                           const real3 *obj_data_C,
                                           const real4 *obj_data_R,
                                           const uint *obj_data_ID,
                                           const bool * obj_active,
                                           const real3 *body_pos,
                                           const real4 *body_rot,
                                           const real & collision_envelope,
                                           long long *contact_pair,
                                           uint *contact_active,
                                           real3 *norm,
                                           real3 *ptA,
                                           real3 *ptB,
                                           real *contactDepth,
                                           int2 *ids

                                           ) {

   long long p = contact_pair[index];
   int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
   uint ID_A = obj_data_ID[pair.x];
   uint ID_B = obj_data_ID[pair.y];

   if (obj_active[ID_A] == false && obj_active[ID_B] == false) {
      return;
   }
   if (ID_A == ID_B) {
      return;
   }

   shape_type A_T = obj_data_T[pair.x], B_T = obj_data_T[pair.y];     //Get the type data for each object in the collision pair
   real3 posA = body_pos[ID_A], posB = body_pos[ID_B];     //Get the global object position
   real4 rotA = (body_rot[ID_A]), rotB = (body_rot[ID_B]);     //Get the global object rotation
   real3 A_X = obj_data_A[pair.x], B_X = obj_data_A[pair.y];
   real3 A_Y = obj_data_B[pair.x], B_Y = obj_data_B[pair.y];
   real3 A_Z = obj_data_C[pair.x], B_Z = obj_data_C[pair.y];
   real4 A_R = (mult(rotA, obj_data_R[pair.x]));
   real4 B_R = (mult(rotB, obj_data_R[pair.y]));

   real envelope = collision_envelope;

   real3 N = R3(1, 0, 0), p1 = R3(0), p2 = R3(0), p0 = R3(0);
   real depth = 0;

   if (A_T == SPHERE && B_T == SPHERE) {
      if (!SphereSphere(A_X, B_X, A_Y, B_Y, N, depth, p1, p2)) {
         return;
      }
   } else {
      if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, N, p0, depth)) {
         return;
      }
//      A_Y -= envelope;
//      B_Y -= envelope;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, N, p0, p1, p2);
   }
   //if(depth>0){swap(p1,p2); depth = -depth;}
   //cout << N << p0 << p1 << p2 << depth << endl;
   depth = dot(N, p2 - p1) + envelope * 2.0;
   p1 = p1 - (N) * envelope;
   p2 = p2 + (N) * envelope;
   //cout << depth << endl;

//   if (depth > envelope) {
//      return;
//   }

   norm[index] = N;
   ptA[index] = p1 - posA;
   ptB[index] = p2 - posB;
   contactDepth[index] = depth;
   ids[index] = I2(ID_A, ID_B);
   contact_active[index] = 0;
}

void ChCNarrowphaseMPR::host_MPR_Store(const shape_type *obj_data_T,
                                       const real3 *obj_data_A,
                                       const real3 *obj_data_B,
                                       const real3 *obj_data_C,
                                       const real4 *obj_data_R,
                                       const uint *obj_data_ID,
                                       const bool * obj_active,
                                       const real3 *body_pos,
                                       const real4 *body_rot,
                                       long long *contact_pair,
                                       uint *contact_active,
                                       real3 *norm,
                                       real3 *ptA,
                                       real3 *ptB,
                                       real *contactDepth,
                                       int2 *ids) {
#pragma omp parallel for
   for (int index = 0; index < total_possible_contacts; index++) {
      function_MPR_Store(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, obj_active, body_pos, body_rot, collision_envelope, contact_pair,
                         contact_active, norm, ptA, ptB, contactDepth, ids);
   }
}

void ChCNarrowphaseMPR::function_MPR_Update(const uint &index,
                                            const shape_type *obj_data_T,
                                            const real3 *obj_data_A,
                                            const real3 *obj_data_B,
                                            const real3 *obj_data_C,
                                            const real4 *obj_data_R,
                                            const uint *obj_data_ID,
                                            const bool * obj_active,
                                            const real3 *body_pos,
                                            const real4 *body_rot,
                                            const real & collision_envelope,
                                            real3 *norm,
                                            real3 *ptA,
                                            real3 *ptB,
                                            real *contactDepth,
                                            int2 *ids) {

   int2 pair = ids[index];
   shape_type A_T = obj_data_T[pair.x], B_T = obj_data_T[pair.y];     //Get the type data for each object in the collision pair
   uint ID_A = obj_data_ID[pair.x];
   uint ID_B = obj_data_ID[pair.y];

   if (obj_active[ID_A] == false && obj_active[ID_B] == false) {
      return;
   }
   if (ID_A == ID_B) {
      return;
   }

   real3 posA = body_pos[ID_A], posB = body_pos[ID_B];     //Get the global object position
   real4 rotA = body_rot[ID_A], rotB = body_rot[ID_B];     //Get the global object rotation
   real3 A_X = obj_data_A[pair.x], B_X = obj_data_A[pair.y];
   real3 A_Y = obj_data_B[pair.x], B_Y = obj_data_B[pair.y];
   real3 A_Z = obj_data_C[pair.x], B_Z = obj_data_C[pair.y];
   real4 A_R = (mult(rotA, obj_data_R[pair.x]));
   real4 B_R = (mult(rotB, obj_data_R[pair.y]));

   real envelope = collision_envelope;

   if (A_T == SPHERE || A_T == ELLIPSOID || A_T == BOX || A_T == CYLINDER || A_T == CONE) {
      A_X = quatRotate(A_X, rotA) + posA;
   } else if (A_T == TRIANGLEMESH) {
      envelope = 0;
      A_X = quatRotate(A_X, rotA) + posA;
      A_Y = quatRotate(A_Y, rotA) + posA;
      A_Z = quatRotate(A_Z, rotA) + posA;
   }

   if (B_T == SPHERE || B_T == ELLIPSOID || B_T == BOX || B_T == CYLINDER || B_T == CONE) {
      B_X = quatRotate(B_X, rotB) + posB;
   } else if (B_T == TRIANGLEMESH) {
      envelope = 0;
      B_X = quatRotate(B_X, rotB) + posB;
      B_Y = quatRotate(B_Y, rotB) + posB;
      B_Z = quatRotate(B_Z, rotB) + posB;
   }

   real3 N = R3(1, 0, 0), p1 = R3(0), p2 = R3(0), p0 = R3(0);
   real depth = 0;

   if (A_T == SPHERE && B_T == SPHERE) {
      if (!SphereSphere(A_X, B_X, A_Y, B_Y, N, depth, p1, p2)) {
         return;
      }
   } else {

      if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, N, p0, depth)) {
         //contactDepth[index] = 0;
         return;
      }

      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, N, p0, p1, p2);

   }

   p1 = p1 - (N) * envelope;
   p2 = p2 + (N) * envelope;

   depth = dot(N, p2 - p1);
   norm[index] = N;
   ptA[index] = p1;
   ptB[index] = p2;

//contactDepth[index] = depth;

}

void ChCNarrowphaseMPR::host_MPR_Update(const shape_type *obj_data_T,
                                        const real3 *obj_data_A,
                                        const real3 *obj_data_B,
                                        const real3 *obj_data_C,
                                        const real4 *obj_data_R,
                                        const uint *obj_data_ID,
                                        const bool * obj_active,
                                        const real3 *body_pos,
                                        const real4 *body_rot,
                                        real3 *norm,
                                        real3 *ptA,
                                        real3 *ptB,
                                        real *contactDepth,
                                        int2 *ids) {
//#pragma omp parallel for
   for (int index = 0; index < total_possible_contacts; index++) {
      function_MPR_Update(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, obj_active, body_pos, body_rot, collision_envelope, norm, ptA, ptB,
                          contactDepth, ids);
   }
}

void host_Preprocess(const uint &index,
                     const shape_type *obj_data_T,
                     const real3 *obj_data_A,
                     const real3 *obj_data_B,
                     const real3 *obj_data_C,
                     const real4 *obj_data_R,
                     const uint *obj_data_ID,
                     const bool * obj_active,
                     const real3 *body_pos,
                     const real4 *body_rot,
                     real3 *obj_data_A_mod,
                     real3 *obj_data_B_mod,
                     real3 *obj_data_C_mod) {

   shape_type T = obj_data_T[index];

   uint ID = obj_data_ID[index];

   real3 pos = body_pos[ID];     //Get the global object position
   real4 rot = body_rot[ID];     //Get the global object rotation
   real3 X = obj_data_A[index];

   if (T == SPHERE || T == ELLIPSOID || T == BOX || T == CYLINDER || T == CONE) {
      obj_data_A_mod[index] = quatRotate(X, rot) + pos;
   } else if (T == TRIANGLEMESH) {
      real3 Y = obj_data_B[index];
      real3 Z = obj_data_C[index];
      obj_data_A_mod[index] = quatRotate(X, rot) + pos;
      obj_data_B_mod[index] = quatRotate(Y, rot) + pos;
      obj_data_C_mod[index] = quatRotate(Z, rot) + pos;
   }

}

void Preprocess(const int numAABB,
                const shape_type *obj_data_T,
                const real3 *obj_data_A,
                const real3 *obj_data_B,
                const real3 *obj_data_C,
                const real4 *obj_data_R,
                const uint *obj_data_ID,
                const bool * obj_active,
                const real3 *body_pos,
                const real4 *body_rot,
                real3 *obj_data_A_mod,
                real3 *obj_data_B_mod,
                real3 *obj_data_C_mod) {
#pragma omp parallel for
   for (int index = 0; index < numAABB; index++) {
      host_Preprocess(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, obj_active, body_pos, body_rot, obj_data_A_mod, obj_data_B_mod,
                      obj_data_C_mod);
   }
}

void ChCNarrowphaseMPR::DoNarrowphase(const custom_vector<shape_type> &obj_data_T,
                                      const custom_vector<real3> &obj_data_A,
                                      const custom_vector<real3> &obj_data_B,
                                      const custom_vector<real3> &obj_data_C,
                                      const custom_vector<real4> &obj_data_R,
                                      const custom_vector<uint> &obj_data_ID,
                                      const custom_vector<bool> & obj_active,
                                      const custom_vector<real3> &body_pos,
                                      const custom_vector<real4> &body_rot,
                                      custom_vector<long long> &potentialCollisions,
                                      custom_vector<real3> &norm_data,
                                      custom_vector<real3> &cpta_data,
                                      custom_vector<real3> &cptb_data,
                                      custom_vector<real> &dpth_data,
                                      custom_vector<int2> &bids_data,
                                      uint & number_of_contacts) {

   total_possible_contacts = potentialCollisions.size();

#if PRINT_LEVEL==2
   cout << "Number of total_possible_contacts: " << total_possible_contacts << endl;
#endif
   custom_vector<uint> generic_counter(total_possible_contacts);
   thrust::fill(generic_counter.begin(), generic_counter.end(), 1);
   norm_data.resize(total_possible_contacts);
   cpta_data.resize(total_possible_contacts);
   cptb_data.resize(total_possible_contacts);
   dpth_data.resize(total_possible_contacts);
   bids_data.resize(total_possible_contacts);

   obj_data_A_mod = obj_data_A;
   obj_data_B_mod = obj_data_B;
   obj_data_C_mod = obj_data_C;

   Preprocess(obj_data_T.size(), obj_data_T.data(), obj_data_A.data(), obj_data_B.data(), obj_data_C.data(), obj_data_R.data(), obj_data_ID.data(), obj_active.data(),
              body_pos.data(), body_rot.data(), obj_data_A_mod.data(), obj_data_B_mod.data(), obj_data_C_mod.data());

   host_MPR_Store(obj_data_T.data(), obj_data_A_mod.data(), obj_data_B_mod.data(), obj_data_C_mod.data(), obj_data_R.data(), obj_data_ID.data(), obj_active.data(), body_pos.data(),
                  body_rot.data(), potentialCollisions.data(), generic_counter.data(), norm_data.data(), cpta_data.data(), cptb_data.data(), dpth_data.data(), bids_data.data());

   number_of_contacts = total_possible_contacts - thrust::count(generic_counter.begin(), generic_counter.end(), 1);
#if PRINT_LEVEL==2
   cout << "Number of number_of_contacts: " << number_of_contacts << endl;
#endif
   thrust::remove_if(norm_data.begin(), norm_data.end(), generic_counter.begin(), thrust::identity<int>());
   thrust::remove_if(cpta_data.begin(), cpta_data.end(), generic_counter.begin(), thrust::identity<int>());
   thrust::remove_if(cptb_data.begin(), cptb_data.end(), generic_counter.begin(), thrust::identity<int>());
   thrust::remove_if(dpth_data.begin(), dpth_data.end(), generic_counter.begin(), thrust::identity<int>());
   thrust::remove_if(bids_data.begin(), bids_data.end(), generic_counter.begin(), thrust::identity<int>());
   thrust::remove_if(potentialCollisions.begin(), potentialCollisions.end(), generic_counter.begin(), thrust::identity<int>());

   potentialCollisions.resize(number_of_contacts);
   norm_data.resize(number_of_contacts);
   cpta_data.resize(number_of_contacts);
   cptb_data.resize(number_of_contacts);
   dpth_data.resize(number_of_contacts);
   bids_data.resize(number_of_contacts);

// thrust::sort_by_key(thrust::omp::par,
// potentialCollisions.begin(),
// potentialCollisions.end(),
// thrust::make_zip_iterator(thrust::make_tuple(norm_data.begin(), cpta_data.begin(), cptb_data.begin(), dpth_data.begin(), bids_data.begin()))
// );

}

void ChCNarrowphaseMPR::UpdateNarrowphase(const custom_vector<shape_type> &obj_data_T,
                                          const custom_vector<real3> &obj_data_A,
                                          const custom_vector<real3> &obj_data_B,
                                          const custom_vector<real3> &obj_data_C,
                                          const custom_vector<real4> &obj_data_R,
                                          const custom_vector<uint> &obj_data_ID,
                                          const custom_vector<bool> & obj_active,
                                          const custom_vector<real3> &body_pos,
                                          const custom_vector<real4> &body_rot,
                                          const uint & number_of_contacts,
                                          custom_vector<real3> &norm_data,
                                          custom_vector<real3> &cpta_data,
                                          custom_vector<real3> &cptb_data,
                                          custom_vector<real> &dpth_data,
                                          custom_vector<int2> &bids_data) {
   total_possible_contacts = number_of_contacts;

   host_MPR_Update(obj_data_T.data(), obj_data_A.data(), obj_data_B.data(), obj_data_C.data(), obj_data_R.data(), obj_data_ID.data(), obj_active.data(), body_pos.data(),
                   body_rot.data(), norm_data.data(), cpta_data.data(), cptb_data.data(), dpth_data.data(), bids_data.data());

}

