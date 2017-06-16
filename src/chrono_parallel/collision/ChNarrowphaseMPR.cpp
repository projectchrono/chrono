#include <algorithm>

#include "chrono/collision/ChCCollisionModel.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChNarrowphaseUtils.h"
#include "chrono_parallel/collision/ChDataStructures.h"

using namespace chrono;
using namespace chrono::collision;

#define MPR_TOLERANCE C_EPSILON
#define MAX_ITERATIONS 100
#define WHILE_LOOP_MAX 1000
struct support {
    real3 v, v1, v2;
};

struct simplex {
    support s0, s1, s2, s3, s4;
};

bool chrono::collision::MPRSphereSphere(const ConvexBase* ShapeA,
                                        const ConvexBase* ShapeB,
                                        real3& N,
                                        real& depth,
                                        real3& p1,
                                        real3& p2) {
    real3 relpos = ShapeB->A() - ShapeA->A();
    real d2 = Dot(relpos);
    real collide_dist = ShapeA->Radius() + ShapeB->Radius();
    if (d2 <= collide_dist * collide_dist) {
        N = relpos / Sqrt(d2);
        p1 = ShapeA->A() + N * ShapeA->Radius();
        p2 = ShapeB->A() - N * ShapeB->Radius();
        depth = Dot(N, p2 - p1);
        return true;
    }
    return false;
}

real3 GetCenter(const ConvexBase* Shape) {
    switch (Shape->Type()) {
        case chrono::collision::TRIANGLEMESH:
            return GetCenter_Triangle(Shape->Triangles());  // triangle center
            break;
        case chrono::collision::CONVEX:
            return GetCenter_Convex(Shape->Size(), Shape->Convex()) + Shape->A();  // convex center
            break;
        case chrono::collision::TETRAHEDRON:
            return GetCenter_Tetrahedron(Shape->TetIndex(), Shape->TetNodes());  // tetrahedron center
            break;
        default:
            return Shape->A();  // All other shapes assumed to be locally centereda
            break;
    }
}

void FindCenter(const ConvexBase* shapeA, const ConvexBase* shapeB, simplex& portal) {
    // v0 = center of Minkowski sum
    portal.s0.v1 = GetCenter(shapeA);
    portal.s0.v2 = GetCenter(shapeB);
    portal.s0.v = portal.s0.v2 - portal.s0.v1;
}

void MPRSupport(const ConvexBase* shapeA, const ConvexBase* shapeB, const real3& n, const real& envelope, support& s) {
    s.v1 = TransformSupportVert(shapeA, -n, envelope);
    s.v2 = TransformSupportVert(shapeB, n, envelope);
    s.v = s.v2 - s.v1;
}

void ExpandPortal(simplex& portal) {
    // Compute the tetrahedron dividing face (v4,v0,v1)
    if (Dot(Cross(portal.s4.v, portal.s1.v), portal.s0.v) < 0) {
        // Compute the tetrahedron dividing face (v4,v0,v2)
        if (Dot(Cross(portal.s4.v, portal.s2.v), portal.s0.v) < 0) {
            portal.s1 = portal.s4;  // Inside d1 & inside d2 ==> eliminate v1
        } else {
            portal.s3 = portal.s4;  // Inside d1 & outside d2 ==> eliminate v3
        }
    } else {
        // Compute the tetrahedron dividing face (v4,v0,v3)
        if (Dot(Cross(portal.s4.v, portal.s3.v), portal.s0.v) < 0) {
            portal.s2 = portal.s4;  // Outside d1 & inside d3 ==> eliminate v2
        } else {
            portal.s1 = portal.s4;  // Outside d1 & outside d3 ==> eliminate v1
        }
    }
}

real3 PortalDir(const simplex& portal) {
    return Normalize(Cross((portal.s2.v - portal.s1.v), (portal.s3.v - portal.s1.v)));
}

void FindPos(const simplex& portal, real3& point) {
    real3 n = PortalDir(portal);
    // Compute the barycentric coordinates of the origin
    real b0 = Dot(Cross(portal.s1.v, portal.s2.v), portal.s3.v);
    real b1 = Dot(Cross(portal.s3.v, portal.s2.v), portal.s0.v);
    real b2 = Dot(Cross(portal.s0.v, portal.s1.v), portal.s3.v);
    real b3 = Dot(Cross(portal.s2.v, portal.s1.v), portal.s0.v);
    real sum = b0 + b1 + b2 + b3;

    if (sum <= 0.0) {
        b0 = 0;
        b1 = Dot(Cross(portal.s2.v, portal.s3.v), n);
        b2 = Dot(Cross(portal.s3.v, portal.s1.v), n);
        b3 = Dot(Cross(portal.s1.v, portal.s2.v), n);
        sum = b1 + b2 + b3;
    }

    real inv = 1.0 / sum;
    real3 p1 = (b0 * portal.s0.v1 + b1 * portal.s1.v1 + b2 * portal.s2.v1 + b3 * portal.s3.v1) * inv;
    real3 p2 = (b0 * portal.s0.v2 + b1 * portal.s1.v2 + b2 * portal.s2.v2 + b3 * portal.s3.v2) * inv;
    point = (p1 + p2) * 0.5;
}

int portalEncapsulesOrigin(const simplex& portal, const real3& n) {
    return Dot(n, portal.s1.v) >= 0.0;
}

int portalCanEncapsuleOrigin(const simplex& portal, const real3& n) {
    return Dot(portal.s4.v, n) >= 0.0;
}

int portalReachTolerance(const simplex& portal, const real3& n) {
    real dv1 = Dot(portal.s1.v, n);
    real dv2 = Dot(portal.s2.v, n);
    real dv3 = Dot(portal.s3.v, n);
    real dv4 = Dot(portal.s4.v, n);

    real dot1 = dv4 - dv1;
    real dot2 = dv4 - dv2;
    real dot3 = dv4 - dv3;

    dot1 = Min(dot1, dot2);
    dot1 = Min(dot1, dot3);

    return IsEqual(dot1, MPR_TOLERANCE) || dot1 < MPR_TOLERANCE;
}
real Vec3Dist2(const real3 a, const real3 b) {
    real3 ab = a - b;
    return Dot(ab, ab);
}
real Vec3PointSegmentDist2(const real3& P, const real3& x0, const real3& b, real3& witness) {
    real dist, t;
    real3 d, a;
    d = b - x0;  // direction of segment
    a = x0 - P;  // precompute vector from P to x0
    t = -(1.) * Dot(a, d);
    t = t / Dot(d, d);

    if (t < 0.0 || IsZero(t)) {
        dist = Vec3Dist2(x0, P);
        witness = x0;
    } else if (t > 1.0 || IsEqual(t, real(1.0))) {
        dist = Vec3Dist2(b, P);
        witness = b;
    } else {
        witness = d;
        witness *= t;
        witness += x0;
        dist = Vec3Dist2(witness, P);
    }

    return dist;
}

real Vec3PointSegmentDist2(const real3& P, const real3& x0, const real3& b) {
    real dist, t;
    real3 d, a;
    d = b - x0;  // direction of segment
    a = x0 - P;  // precompute vector from P to x0
    t = -(1.) * Dot(a, d);
    t = t / Dot(d, d);

    if (t < 0.0 || IsZero(t)) {
        dist = Vec3Dist2(x0, P);

    } else if (t > 1.0 || IsEqual(t, real(1.0))) {
        dist = Vec3Dist2(b, P);

    } else {
        d = d * t;
        d = d * a;
        dist = Dot(d, d);
    }

    return dist;
}

real Vec3PointTriDist2(const real3& P, const real3& x0, const real3& B, const real3& C, real3& witness) {
    real3 d1, d2, a;
    real u, v, w, p, q, r;
    real s, t, dist, dist2;
    real3 witness2;

    d1 = B - x0;
    d2 = C - x0;
    a = x0 - P;

    u = Dot(a, a);
    v = Dot(d1, d1);
    w = Dot(d2, d2);
    p = Dot(a, d1);
    q = Dot(a, d2);
    r = Dot(d1, d2);

    s = (q * r - w * p) / (w * v - r * r);
    t = (-s * r - q) / w;
    if ((IsZero(s) || s > 0.0) && (IsEqual(s, real(1.0)) || s < real(1.0)) && (IsZero(t) || t > 0.0) &&
        (IsEqual(t, real(1.0)) || t < real(1.0)) && (IsEqual(t + s, real(1.0)) || t + s < real(1.0))) {
        d1 *= s;
        d2 *= t;
        witness = x0;
        witness += d1;
        witness += d2;

        dist = Vec3Dist2(witness, P);
    } else {
        dist = Vec3PointSegmentDist2(P, x0, B, witness);
        dist2 = Vec3PointSegmentDist2(P, x0, C, witness2);
        if (dist2 < dist) {
            dist = dist2;
            witness = witness2;
        }
        dist2 = Vec3PointSegmentDist2(P, B, C);
        if (dist2 < dist) {
            dist = dist2;
            witness = witness2;
        }
    }
    return dist;
}

real Vec3PointTriDist2(const real3& P, const real3& V0, const real3& V1, const real3& V2) {
    real3 diff = V0 - P;
    real3 edge0 = V1 - V0;
    real3 edge1 = V2 - V0;
    real a00 = Dot(edge0, edge0);
    real a01 = Dot(edge0, edge1);
    real a11 = Dot(edge1, edge1);
    real b0 = Dot(diff, edge0);
    real b1 = Dot(diff, edge1);
    real c = Dot(diff, diff);
    real det = Abs(a00 * a11 - a01 * a01);
    real s = a01 * b1 - a11 * b0;
    real t = a01 * b0 - a00 * b1;
    real sqrDistance;

    if (s + t <= det) {
        if (s < (real)0) {
            if (t < (real)0)  // region 4
            {
                if (b0 < (real)0) {
                    t = (real)0;
                    if (-b0 >= a00) {
                        s = (real)1;
                        sqrDistance = a00 + ((real)2) * b0 + c;
                    } else {
                        s = -b0 / a00;
                        sqrDistance = b0 * s + c;
                    }
                } else {
                    s = (real)0;
                    if (b1 >= (real)0) {
                        t = (real)0;
                        sqrDistance = c;
                    } else if (-b1 >= a11) {
                        t = (real)1;
                        sqrDistance = a11 + ((real)2) * b1 + c;
                    } else {
                        t = -b1 / a11;
                        sqrDistance = b1 * t + c;
                    }
                }
            } else  // region 3
            {
                s = (real)0;
                if (b1 >= (real)0) {
                    t = (real)0;
                    sqrDistance = c;
                } else if (-b1 >= a11) {
                    t = (real)1;
                    sqrDistance = a11 + ((real)2) * b1 + c;
                } else {
                    t = -b1 / a11;
                    sqrDistance = b1 * t + c;
                }
            }
        } else if (t < (real)0)  // region 5
        {
            t = (real)0;
            if (b0 >= (real)0) {
                s = (real)0;
                sqrDistance = c;
            } else if (-b0 >= a00) {
                s = (real)1;
                sqrDistance = a00 + ((real)2) * b0 + c;
            } else {
                s = -b0 / a00;
                sqrDistance = b0 * s + c;
            }
        } else  // region 0
        {
            // minimum at interior point
            real invDet = ((real)1) / det;
            s *= invDet;
            t *= invDet;
            sqrDistance = s * (a00 * s + a01 * t + ((real)2) * b0) + t * (a01 * s + a11 * t + ((real)2) * b1) + c;
        }
    } else {
        real tmp0, tmp1, numer, denom;

        if (s < (real)0)  // region 2
        {
            tmp0 = a01 + b0;
            tmp1 = a11 + b1;
            if (tmp1 > tmp0) {
                numer = tmp1 - tmp0;
                denom = a00 - ((real)2) * a01 + a11;
                if (numer >= denom) {
                    s = (real)1;
                    t = (real)0;
                    sqrDistance = a00 + ((real)2) * b0 + c;
                } else {
                    s = numer / denom;
                    t = (real)1 - s;
                    sqrDistance =
                        s * (a00 * s + a01 * t + ((real)2) * b0) + t * (a01 * s + a11 * t + ((real)2) * b1) + c;
                }
            } else {
                s = (real)0;
                if (tmp1 <= (real)0) {
                    t = (real)1;
                    sqrDistance = a11 + ((real)2) * b1 + c;
                } else if (b1 >= (real)0) {
                    t = (real)0;
                    sqrDistance = c;
                } else {
                    t = -b1 / a11;
                    sqrDistance = b1 * t + c;
                }
            }
        } else if (t < (real)0)  // region 6
        {
            tmp0 = a01 + b1;
            tmp1 = a00 + b0;
            if (tmp1 > tmp0) {
                numer = tmp1 - tmp0;
                denom = a00 - ((real)2) * a01 + a11;
                if (numer >= denom) {
                    t = (real)1;
                    s = (real)0;
                    sqrDistance = a11 + ((real)2) * b1 + c;
                } else {
                    t = numer / denom;
                    s = (real)1 - t;
                    sqrDistance =
                        s * (a00 * s + a01 * t + ((real)2) * b0) + t * (a01 * s + a11 * t + ((real)2) * b1) + c;
                }
            } else {
                t = (real)0;
                if (tmp1 <= (real)0) {
                    s = (real)1;
                    sqrDistance = a00 + ((real)2) * b0 + c;
                } else if (b0 >= (real)0) {
                    s = (real)0;
                    sqrDistance = c;
                } else {
                    s = -b0 / a00;
                    sqrDistance = b0 * s + c;
                }
            }
        } else  // region 1
        {
            numer = a11 + b1 - a01 - b0;
            if (numer <= (real)0) {
                s = (real)0;
                t = (real)1;
                sqrDistance = a11 + ((real)2) * b1 + c;
            } else {
                denom = a00 - ((real)2) * a01 + a11;
                if (numer >= denom) {
                    s = (real)1;
                    t = (real)0;
                    sqrDistance = a00 + ((real)2) * b0 + c;
                } else {
                    s = numer / denom;
                    t = (real)1 - s;
                    sqrDistance =
                        s * (a00 * s + a01 * t + ((real)2) * b0) + t * (a01 * s + a11 * t + ((real)2) * b1) + c;
                }
            }
        }
    }

    // Account for numerical round-off error.
    if (sqrDistance < (real)0) {
        sqrDistance = (real)0;
    }

    // mClosestPoint0 = P;
    // mClosestPoint1 = V0 + s * edge0 + t * edge1;
    // mTriangleBary[1] = s;
    // mTriangleBary[2] = t;
    // mTriangleBary[0] = (real) 1 - s - t;
    return sqrDistance;
}

void FindPenetration(const ConvexBase* shapeA,
                     const ConvexBase* shapeB,
                     const real& envelope,
                     simplex& portal,
                     real& depth,
                     real3& n,
                     real3& point) {
    real3 zero = real3(0);
    real3 dir;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        // std::cout<<i<<std::endl;
        dir = PortalDir(portal);
        MPRSupport(shapeA, shapeB, dir, envelope, portal.s4);

        real delta = Dot((portal.s4.v - portal.s3.v), dir);
        // std::cout<<dir<<" "<<delta<<std::endl;
        if (portalReachTolerance(portal, dir) || i == MAX_ITERATIONS - 1) {
            //         depth = -Sqrt(Vec3PointTriDist2(zero, portal.s1.v, portal.s2.v, portal.s3.v, n));
            //         if (IsZero(n)) {
            //            n = dir;
            //         }
            n = Normalize(dir);
            FindPos(portal, point);

            return;
        }
        ExpandPortal(portal);
    }
}

void FindPenetrationTouch(const ConvexBase* shapeA,
                          const ConvexBase* shapeB,
                          simplex& portal,
                          real& depth,
                          real3& n,
                          real3& point) {
    depth = 0;
    n = Normalize(portal.s1.v - portal.s0.v);
    point = (portal.s1.v1 + portal.s1.v2) * .5;
}

void FindPenetrationSegment(const ConvexBase* shapeA,
                            const ConvexBase* shapeB,
                            simplex& portal,
                            real& depth,
                            real3& n,
                            real3& point) {
    point = (portal.s1.v1 + portal.s1.v2) * .5;
    n = portal.s1.v;
    depth = -Length(n);
    n = Normalize(n);
}

bool FindPortal(const ConvexBase* shapeA, const ConvexBase* shapeB, const real& envelope, simplex& portal, real3& n) {
    // Phase One: Identify a portal
    for (int wi = 0; wi < WHILE_LOOP_MAX; wi++) {
        // Obtain the support point in a direction perpendicular to the existing plane
        // Note: This point is guaranteed to lie off the plane
        MPRSupport(shapeA, shapeB, n, envelope, portal.s3);

        if (Dot(portal.s3.v, n) <= 0.0) {
            // cout << "FAIL C" << endl;
            return false;
        }
        // If origin is outside (v1,v0,v3), then eliminate v2 and loop
        if (Dot(Cross(portal.s1.v, portal.s3.v), portal.s0.v) < 0.0) {
            portal.s2 = portal.s3;
            n = Normalize(Cross((portal.s1.v - portal.s0.v), (portal.s3.v - portal.s0.v)));
            continue;
        }
        // If origin is outside (v3,v0,v2), then eliminate v1 and loop
        if (Dot(Cross(portal.s3.v, portal.s2.v), portal.s0.v) < 0.0) {
            portal.s1 = portal.s3;
            n = Normalize(Cross((portal.s3.v - portal.s0.v), (portal.s2.v - portal.s0.v)));
            continue;
        }
        break;
    }
    return true;
}

int DiscoverPortal(const ConvexBase* shapeA, const ConvexBase* shapeB, const real& envelope, simplex& portal) {
    real3 n, va, vb;
    // vertex 0 is center of portal
    FindCenter(shapeA, shapeB, portal);

    // Avoid case where centers overlap -- any direction is fine in this case
    if (IsZero(portal.s0.v)) {
        portal.s0.v = real3(1, 0, 0);
    }
    // v1 = support in direction of origin
    n = Normalize(-portal.s0.v);
    MPRSupport(shapeA, shapeB, n, envelope, portal.s1);
    // test if origin isn't outside of v1
    if (Dot(portal.s1.v, n) < 0.0) {
        // no contact
        // cout << "FAIL A " << Dot(portal.s1.v, n) << endl;
        return -1;
    }

    // v2 - support perpendicular to v1,v0
    n = Cross(portal.s0.v, portal.s1.v);

    if (IsZero(n)) {
        if (IsZero(portal.s1.v)) {
            return 1;
        } else {
            return 2;
        }
    }
    n = Normalize(n);

    MPRSupport(shapeA, shapeB, n, envelope, portal.s2);
    if (Dot(portal.s2.v, n) <= 0.0) {
        return -1;
    }

    // Determine whether origin is on + or - side of plane (v1,v0,v2)
    n = Normalize(Cross((portal.s1.v - portal.s0.v), (portal.s2.v - portal.s0.v)));
    // If the origin is on the - side of the plane, reverse the direction of the plane
    if (Dot(n, portal.s0.v) > 0.0) {
        Swap(portal.s1, portal.s2);
        n = -n;
    }
    //int cont;
    // FindPortal code
    for (int wi = 0; wi < WHILE_LOOP_MAX; wi++) {
        // Obtain the support point in a direction perpendicular to the existing plane
        // Note: This point is guaranteed to lie off the plane
        MPRSupport(shapeA, shapeB, n, envelope, portal.s3);

        if (Dot(portal.s3.v, n) <= 0.0) {
            return -1;
        }
        // If origin is outside (v1,v0,v3), then eliminate v2 and loop
        if (Dot(Cross(portal.s1.v, portal.s3.v), portal.s0.v) < 0.0) {
            portal.s2 = portal.s3;
            n = Normalize(Cross((portal.s1.v - portal.s0.v), (portal.s3.v - portal.s0.v)));
            continue;
        }
        // If origin is outside (v3,v0,v2), then eliminate v1 and loop
        if (Dot(Cross(portal.s3.v, portal.s2.v), portal.s0.v) < 0.0) {
            portal.s1 = portal.s3;
            n = Normalize(Cross((portal.s3.v - portal.s0.v), (portal.s2.v - portal.s0.v)));
            continue;
        }
        break;
    }
    return 0;
}
int RefinePortal(const ConvexBase* shapeA, const ConvexBase* shapeB, const real& envelope, simplex& portal) {
    real3 n;
    for (int i = 0; i < MAX_ITERATIONS; i++) {
        // Compute normal of the wedge face
        n = PortalDir(portal);

        if (portalEncapsulesOrigin(portal, n)) {
            return 0;
        }

        // Find the support point in the direction of the wedge face
        MPRSupport(shapeA, shapeB, n, envelope, portal.s4);

        // If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex,
        // then
        // we can terminate
        if (portalReachTolerance(portal, n) || !portalCanEncapsuleOrigin(portal, n)) {
            if (portalEncapsulesOrigin(portal, n)) {
                return 0;
            }
            return -1;
        }
        ExpandPortal(portal);
    }
    return -1;
}
// Code for Convex-Convex Collision detection, adopted from xeno-collide
bool chrono::collision::MPRContact(const ConvexBase* shapeA,
                                   const ConvexBase* shapeB,
                                   const real& envelope,
                                   real3& returnNormal,
                                   real3& point,
                                   real& depth) {
    simplex portal;

    int result = DiscoverPortal(shapeA, shapeB, envelope, portal);
    // std::cout << result << std::endl;

    if (result == 0) {
        result = RefinePortal(shapeA, shapeB, envelope, portal);
        // std::cout << result << std::endl;

        if (result < 0) {
            return 0;
        }
        FindPenetration(shapeA, shapeB, envelope, portal, depth, returnNormal, point);
    } else if (result == 1) {
        FindPenetrationTouch(shapeA, shapeB, portal, depth, returnNormal, point);
    } else if (result == 2) {
        FindPenetrationSegment(shapeA, shapeB, portal, depth, returnNormal, point);
    } else {
        return 0;
    }
    return 1;
}

void chrono::collision::MPRGetPoints(const ConvexBase* shapeA,
                                     const ConvexBase* shapeB,
                                     const real& envelope,
                                     real3& N,
                                     real3 p0,
                                     real3& p1,
                                     real3& p2) {
    p1 = Dot((TransformSupportVert(shapeA, -N, envelope) - p0), N) * N + p0;
    p2 = Dot((TransformSupportVert(shapeB, N, envelope) - p0), N) * N + p0;
    N = -N;
}

// Code for Convex-Convex Collision detection, adopted from xeno-collide
bool chrono::collision::MPRCollision(const ConvexBase* shapeA,
                                     const ConvexBase* shapeB,
                                     real envelope,
                                     real3& normal,
                                     real3& pointA,
                                     real3& pointB,
                                     real& depth) {
    real3 point;
    if (!MPRContact(shapeA, shapeB, envelope, normal, point, depth)) {
        return false;
    }

    MPRGetPoints(shapeA, shapeB, envelope, normal, point, pointA, pointB);

    pointA = pointA - normal * envelope;
    pointB = pointB + normal * envelope;
    depth = Dot(normal, pointB - pointA);
    return true;
}
