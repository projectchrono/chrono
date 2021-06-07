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

#include "chrono/collision/ChCollisionUtilsBullet.h"
#include "chrono/collision/bullet/LinearMath/btConvexHull.h"

namespace chrono {
namespace collision {
namespace utils {

// -----------------------------------------------------------------------------

// Project point onto line.
btVector3 ProjectPointOnLine(const btVector3& lP,  // point on line
                             const btVector3& lD,  // line direction (unit vector)
                             const btVector3& P    // point
) {
    return lP + (P - lP).dot(lD) * lD;
}

btScalar DistancePointToLine(const btVector3& lP,  // point on line
                             const btVector3& lD,  // line direction (unit vector)
                             const btVector3& P    // point
) {
    btVector3 Q = ProjectPointOnLine(lP, lD, P);
    return (Q - P).length();
}

// -----------------------------------------------------------------------------

// Snap the specified location to a point on a box with given half-dimensions.
// The in/out location is assumed to be specified in the frame of the box (which is therefore assumed to be an AABB
// centered at the origin).  The return code indicates the box axes that caused snapping:
//   - first bit (least significant) corresponds to x-axis
//   - second bit corresponds to y-axis
//   - third bit corresponds to z-axis
//
// Therefore:
//   code = 0 indicates an interior point
//   code = 1 or code = 2 or code = 4  indicates snapping to a face
//   code = 3 or code = 5 or code = 6  indicates snapping to an edge
//   code = 7 indicates snapping to a corner
int SnapPointToBox(const btVector3& hdims,  // box half-dimensions
                   btVector3& loc           // point (in/out)
) {
    int code = 0;

    if (std::abs(loc[0]) > hdims[0]) {
        code |= 1;
        loc[0] = (loc[0] > 0) ? hdims[0] : -hdims[0];
    }
    if (std::abs(loc[1]) > hdims[1]) {
        code |= 2;
        loc[1] = (loc[1] > 0) ? hdims[1] : -hdims[1];
    }
    if (std::abs(loc[2]) > hdims[2]) {
        code |= 4;
        loc[2] = (loc[2] > 0) ? hdims[2] : -hdims[2];
    }

    return code;
}

// -----------------------------------------------------------------------------

// Check if given point is inside box (point expressed in box frame).
bool PointInsideBox(const btVector3& hdims,  // box half-dimensions
                    const btVector3& loc     // point
) {
    for (int i = 0; i < 3; i++) {
        if (loc[i] > hdims[i] || loc[i] < -hdims[i])
            return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

// Find the closest box face to the given point (expressed in box frame).
// Returns +1, +2, +3 (for a "positive" face in x, y, z, respectively) or -1, -2, -3 (for a "negative" face).
int FindClosestBoxFace(const btVector3& hdims,  // box half-dimensions
                       const btVector3& loc     // point
) {
    int code = 0;
    btScalar dist = +BT_LARGE_FLOAT;
    for (int i = 0; i < 3; i++) {
        btScalar p_dist = std::abs(loc[i] - hdims[i]);
        btScalar n_dist = std::abs(loc[i] + hdims[i]);
        if (p_dist < dist) {
            code = i + 1;
            dist = p_dist;
        }
        if (n_dist < dist) {
            code = -(i + 1);
            dist = n_dist;
        }
    }

    return code;
}

// -----------------------------------------------------------------------------

// Utility function for intersecting a box with a line segment.
// It is assumed that the box is centered at the origin and the segment is expressed in the box frame.
// The function returns false if the segment does not intersect the box.
// Algorithm:
//  - check intersection of the supporting line with the box slabs in the 3 directions
//  - if segment parallel to a slab, no intersection if segment far from box
//  - otherwise, update line parameters with points where line intersects box faces
//    (keep track of the most "inner" points at all times)
//  - finally, clamp to segment length
bool IntersectSegmentBox(const btVector3& hdims,  // box half-dimensions
                         const btVector3& c,      // segment center point
                         const btVector3& a,      // segment direction (unit vector)
                         const btScalar hlen,     // segment half-length
                         const btScalar tol,      // tolerance for parallelism test
                         btScalar& tMin,          // segment parameter of first intersection point
                         btScalar& tMax           // segment parameter of second intersection point
) {
    tMin = -BT_LARGE_FLOAT;
    tMax = +BT_LARGE_FLOAT;

    if (std::abs(a.x()) < tol) {
        // Segment parallel to the box x-faces
        if (std::abs(c.x()) > hdims.x())
            return false;
    } else {
        btScalar t1 = (-hdims.x() - c.x()) / a.x();
        btScalar t2 = (+hdims.x() - c.x()) / a.x();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return false;
    }

    if (std::abs(a.y()) < tol) {
        // Segment parallel to the box y-faces
        if (std::abs(c.y()) > hdims.y())
            return false;
    } else {
        btScalar t1 = (-hdims.y() - c.y()) / a.y();
        btScalar t2 = (+hdims.y() - c.y()) / a.y();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return false;
    }

    if (std::abs(a.z()) < tol) {
        // Capsule axis parallel to the box z-faces
        if (std::abs(c.z()) > hdims.z())
            return false;
    } else {
        btScalar t1 = (-hdims.z() - c.z()) / a.z();
        btScalar t2 = (+hdims.z() - c.z()) / a.z();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return false;
    }

    // If both intersection points are outside the segment, no intersection
    if ((tMin < -hlen && tMax < -hlen) || (tMin > +hlen && tMax > +hlen))
        return false;

    // Clamp intersection points to segment length
    ChClampValue(tMin, -hlen, +hlen);
    ChClampValue(tMax, -hlen, +hlen);

    return true;
}

// -----------------------------------------------------------------------------

// Utility function to intersect a line with a plane
// Plane equation: pN.X = pN.pP
// Line equation:  X = lP + t * lD
// Solution:       t = pN.(pP-lP) / pN.lD
bool IntersectLinePlane(const btVector3& lP,  // point on line
                        const btVector3& lD,  // line direction (unit vector)
                        const btVector3& pP,  // point on plane
                        const btVector3& pN,  // plane normal (unit vector)
                        const btScalar tol,   // tolerance for orthogonality test
                        btScalar& t           // line parameter of intersection point
) {
    btScalar nd = pN.dot(lD);

    if (std::abs(nd) < tol) {
        // Line parallel to plane
        return false;
    }

    t = pN.dot(pP - lP) / nd;
    return true;
}

// -----------------------------------------------------------------------------

// Utility function to intersect a segment with a cylinder.
// Segment assumed to be parameterized as sP = sC + t * sD, with -sH <= t <= sH.
// Cylinder given by its center cC, axis direction cD, halh-lengh cH, and radius cR.
// Assume |sD| = |cD| = 1.
// (1) Find tMin and tMax where the segment supporting line intersects cylindrical surface by finding
//     points on segment at a distance cR from the cylinder axis line.
// (2) Clamp result to cylinder end-caps.
// (3) Clamp result to segment length.
//
// Distance between a point P and cylinder axis:
//    d = |(sP-cC) x cD| / |cD| = |(sP-cC) x cD|
// using |cD|=1.
// Solve:
//    cR^2 = |(sP-cC) x cD|^2 = |sP-cC|^2 * |cD|^2 - [cD.(sP-cC)]^2 = |sP-cC|^2 - [cD.(sP-cC)]^2
//    cR^2 = |sC-cC+t*sD|^2 - [cD.(sC-cC+t*sD)]^2
//       0 = t^2 [ 1 - (cD.sD)^2 ] + 2t [ (v.sD) - (cD.v)(cD.sD) ] + v.v - (cD.v)^2 - cR^2
//       0 = a t^2 + 2bt + c
// where v = sC-cC and using |sD|=1.
bool IntersectSegmentCylinder(const btVector3& sC,  // segment center point
                              const btVector3& sD,  // segment direction (unit vector)
                              const btScalar sH,    // segment half-length
                              const btVector3& cC,  // cylinder axis center
                              const btVector3& cD,  // cylinder axis direction (unit vector)
                              const btScalar cH,    // cylinder axis half-length (cylinder halh-height)
                              const btScalar cR,    // cylinder radius
                              const btScalar tol,   // tolerance for parallelism test
                              btScalar& tMin,       // segment parameter of first intersection point
                              btScalar& tMax        // segment parameter of second intersection point
) {
    tMin = -BT_LARGE_FLOAT;
    tMax = +BT_LARGE_FLOAT;

    btVector3 v = sC - cC;
    btScalar cDsD = cD.dot(sD);
    btScalar vcD = v.dot(cD);
    btScalar vsD = v.dot(sD);
    btScalar vv = v.dot(v);
    btScalar a = 1 - cDsD * cDsD;
    btScalar b = vsD - vcD * cDsD;
    btScalar c = vv - vcD * vcD - cR * cR;

    // Intersection with cylindrical surface.
    // a >= 0 always
    // a == 0 indicates line parallel to cylinder axis
    if (std::abs(a) < tol) {
        // line parallel to cylinder axis
        btScalar dist2 = (v - vcD * cD).length2();
        if (dist2 > cR * cR)
            return false;
        tMin = -sH;
        tMax = +sH;
    } else {
        // line intersects cylindrical surface
        btScalar discr = b * b - a * c;
        if (discr < 0)
            return false;  // no real roots, no intersection
        discr = btSqrt(discr);
        tMin = (-b - discr) / a;
        tMax = (-b + discr) / a;
    }

    // Intersection with end-caps.
    btScalar t1;
    bool code1 = IntersectLinePlane(sC, sD, cC + cH * cD, cD, tol, t1);
    btScalar t2;
    bool code2 = IntersectLinePlane(sC, sD, cC - cH * cD, cD, tol, t2);
    assert(code1 == code2);
    if (code1) {
        // line intersects end-caps
        if (t1 < t2) {
            tMin = btMax(tMin, t1);
            tMax = btMin(tMax, t2);
        } else {
            tMin = btMax(tMin, t2);
            tMax = btMin(tMax, t1);        
        }
        if (tMax < tMin)
            return false;
    } else {
        // line parallel to end-cap planes
        btScalar d1 = std::abs(cD.dot(cC + cH * cD - sC));
        btScalar d2 = std::abs(cD.dot(cC - cH * cD - sC));
        if (d1 > 2 * cH || d2 > 2 * cH)
            return false;
    }

    // If both intersection points are outside the segment, no intersection
    if ((tMin < -sH && tMax < -sH) || (tMin > +sH && tMax > +sH))
        return false;

    // Clamp to segment length
    btClamp(tMin, -sH, +sH);
    btClamp(tMax, -sH, +sH);

    assert(tMin <= tMax);

    return true;
}

// -----------------------------------------------------------------------------

ChConvexHullLibraryWrapper::ChConvexHullLibraryWrapper() {}

void ChConvexHullLibraryWrapper::ComputeHull(const std::vector<ChVector<> >& points,
                                             geometry::ChTriangleMeshConnected& vshape) {
    HullLibrary hl;
    HullResult hresult;
    HullDesc desc;

    desc.SetHullFlag(QF_TRIANGLES);

    btVector3* btpoints = new btVector3[points.size()];
    for (unsigned int ip = 0; ip < points.size(); ++ip) {
        btpoints[ip].setX((btScalar)points[ip].x());
        btpoints[ip].setY((btScalar)points[ip].y());
        btpoints[ip].setZ((btScalar)points[ip].z());
    }
    desc.mVcount = (unsigned int)points.size();
    desc.mVertices = btpoints;
    desc.mVertexStride = sizeof(btVector3);

    HullError hret = hl.CreateConvexHull(desc, hresult);

    if (hret == QE_OK) {
        vshape.Clear();

        vshape.getIndicesVertexes().resize(hresult.mNumFaces);
        for (unsigned int it = 0; it < hresult.mNumFaces; ++it) {
            vshape.getIndicesVertexes()[it] = ChVector<int>(
                hresult.m_Indices[it * 3 + 0], hresult.m_Indices[it * 3 + 1], hresult.m_Indices[it * 3 + 2]);
        }
        vshape.getCoordsVertices().resize(hresult.mNumOutputVertices);
        for (unsigned int iv = 0; iv < hresult.mNumOutputVertices; ++iv) {
            vshape.getCoordsVertices()[iv] = ChVector<>(
                hresult.m_OutputVertices[iv].x(), hresult.m_OutputVertices[iv].y(), hresult.m_OutputVertices[iv].z());
        }
    }

    delete[] btpoints;

    hl.ReleaseResult(hresult);
}

}  // namespace utils
}  // namespace collision
}  // namespace chrono
