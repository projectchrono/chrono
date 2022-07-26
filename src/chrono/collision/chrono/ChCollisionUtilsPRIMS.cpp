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
//
// Header file for ChCNarrowphaseRUtils.
// This file defines various low-level utility functions used in specialized
// pair-wise collision detection (e.g., finding the closest point on a shape to
// a specified point).
//
// =============================================================================

#include <cassert>

#include "chrono/collision/chrono/ChCollisionUtils.h"

namespace chrono {
namespace collision {
namespace ch_utils {

// -----------------------------------------------------------------------------
// Utilities for triangle collisions
// -----------------------------------------------------------------------------

// This utility function takes the location 'P' and snaps it to the closest
// point on the triangular face with given vertices (A, B, and C). The result
// is returned in 'res'. Both 'P' and 'res' are assumed to be specified in
// the same frame as the face vertices. This function returns 'true' if the
// result is on an edge of this face and 'false' if the result is inside the
// triangle.
// Code from Ericson, "Real-time collision detection", 2005, pp. 141
bool snap_to_triangle(const real3& A, const real3& B, const real3& C, const real3& P, real3& res) {
    real3 AB = B - A;
    real3 AC = C - A;

    // Check if P in vertex region outside A
    real3 AP = P - A;
    real d1 = Dot(AB, AP);
    real d2 = Dot(AC, AP);
    if (d1 <= 0 && d2 <= 0) {
        res = A;  // barycentric coordinates (1,0,0)
        return true;
    }

    // Check if P in vertex region outside B
    real3 BP = P - B;
    real d3 = Dot(AB, BP);
    real d4 = Dot(AC, BP);
    if (d3 >= 0 && d4 <= d3) {
        res = B;  // barycentric coordinates (0,1,0)
        return true;
    }

    // Check if P in edge region of AB
    real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        // Return projection of P onto AB
        real v = d1 / (d1 - d3);
        res = A + v * AB;  // barycentric coordinates (1-v,v,0)
        return true;
    }

    // Check if P in vertex region outside C
    real3 CP = P - C;
    real d5 = Dot(AB, CP);
    real d6 = Dot(AC, CP);
    if (d6 >= 0 && d5 <= d6) {
        res = C;  // barycentric coordinates (0,0,1)
        return true;
    }

    // Check if P in edge region of AC
    real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        // Return projection of P onto AC
        real w = d2 / (d2 - d6);
        res = A + w * AC;  // barycentric coordinates (1-w,0,w)
        return true;
    }

    // Check if P in edge region of BC
    real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        // Return projection of P onto BC
        real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        res = B + w * (C - B);  // barycentric coordinates (0,1-w,w)
        return true;
    }

    // P inside face region. Return projection of P onto face
    // barycentric coordinates (u,v,w)
    real denom = 1 / (va + vb + vc);
    real v = vb * denom;
    real w = vc * denom;
    res = A + v * AB + w * AC;  // = u*A + v*B + w*C  where  (u = 1 - v - w)
    return false;
}

bool point_in_triangle(const real3& A, const real3& B, const real3& C, const real3& loc) {
    // Compute vectors
    real3 v0 = C - A;
    real3 v1 = B - A;
    real3 v2 = loc - A;

    // Compute dot products
    real dot00 = Dot(v0, v0);
    real dot01 = Dot(v0, v1);
    real dot02 = Dot(v0, v2);
    real dot11 = Dot(v1, v1);
    real dot12 = Dot(v1, v2);

    // Compute barycentric coordinates
    real invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    real u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    real v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is in triangle
    real tol = 10 * C_REAL_EPSILON;
    return (u > -tol) && (v > -tol) && (u + v < 1 + tol);
}

// -----------------------------------------------------------------------------
// Utilities for box collisions
// -----------------------------------------------------------------------------

// This utility function returns true if 'point' is no farther than 'separation' from the face of this box identified
// by 'pt_on_face' and 'code'; and returns false otherwise. If the point is close enough, its projection onto the box
// face is returned in 'result' and the height of the point relative to the box face is returned in 'dist' (with a
// negative value indicating penetration, i.e. the point inside the box). 'normal' is set to the box face (pointing
// outwards). See box_closest_feature for definition of 'code.
bool point_vs_face(const real3& hdims,
                   const real3& pt_on_face,
                   uint code,
                   const real3& point,
                   const real& separation,
                   real3& result,
                   real3& normal,
                   real& dist) {
    assert(code == 1 || code == 2 || code == 4);
    uint i = code >> 1;     // index of face normal direction (0: x, 1: y, 2: z)
    uint in = (i + 1) % 3;  // next direction (0->1->2->0)
    uint ip = (i + 2) % 3;  // previous direction (0->2->1->0)

    // No interaction if point outside box slabs in face plane
    if (abs(point[in]) > hdims[in])
        return false;
    if (abs(point[ip]) > hdims[ip])
        return false;

    // Decide if working with the "positive" or "negative" box face
    int sign = (pt_on_face[i] > 0 ? +1 : -1);

    // Distance from point to face (negative if point inside box)
    dist = sign * point[i] - hdims[i];
    if (dist > separation)
        return false;

    // Projection of point onto box face
    result = point;
    result[i] = sign * hdims[i];

    // Face normal
    normal[i] = sign;
    normal[in] = 0;
    normal[ip] = 0;

    return true;
}

// This utility function calculates the closest points between the box edge identified by 'pt_on_edge' and 'code' and a
// line segment between 'pt1' and 'pt2'. It returns true if the closest points are within the extent of the edge and
// the segment, respectively, and false otherwise. The closest points are returned in 'locE' and 'locS', respectively.
// This function uses the parametric solution of the two support lines to solve for the closest points, simplified as
// we work in the local frame of the box.
// See box_closest_feature for definition of 'code.
bool segment_vs_edge(const real3& hdims,
                     const real3& pt_on_edge,
                     uint code,
                     const real3& pt1,
                     const real3& pt2,
                     real3& locE,
                     real3& locS) {
    assert(code == 3 || code == 5 || code == 6);
    uint i = (~code & 7) >> 1;  // index of edge direction (0: x, 1: y, 2: z)
    ////uint in = (i + 1) % 3;      // next direction (0->1->2->0)
    ////uint ip = (i + 2) % 3;      // previous direction (0->2->1->0)

    // Vector along segment
    real3 s = pt2 - pt1;
    real e = Dot(s, s);

    // No interaction if segment is parallel to edge.
    // Declare parallel, if angle less than about 1 degree.
    // Note:  sin^2 = d/e and sin^2(1) = 3.046e-4.
    //// TODO: this should be treated separately!
    real d = e - s[i] * s[i];
    if (d < real(3e-4) * e)
        return false;

    // Closest point on segment (line parameter tS between 0 and 1)
    real3 r = pt_on_edge - pt1;
    real tS = (Dot(s, r) - s[i] * r[i]) / d;

    // No interaction if point not on segment.
    // Use a tolerance a fraction of the segment length.
    real eps = real(1e-8) * e;
    if (tS < eps || tS > 1 - eps)
        return false;

    // Closest point on box edge (line parameter tE between 0 and 2*hdims[i])
    real tE = s[i] * tS - r[i];
    locE = pt_on_edge;
    locE[i] += tE;

    // No interaction if point not on edge
    if (abs(locE[i]) > hdims[i])
        return false;

    // Closest point on segment
    locS = pt1 + s * tS;

    return true;
}

// This function returns an integer indicating whether or not a box1 with
// dimensions hdims1 intersects (or is close enough to) a second box with
// dimensions hdims2.
// The check is performed in the local frame of box1. The transform from the
// other box is given through 'pos' and 'rot'.
//
// The return value is -1 if the two boxes overlap, +1 if they are within
// a distance of 'separation' from each other, and 0 if they are "far" from
// each other.
// If returning -1 or +1, 'dir' contains the direction of smallest intersection
// (or closest separation).
//
// This check is performed by testing 15 possible separating planes between the
// two boxes (Gottschalk, Lin, Manocha - Siggraph96).
//
// If not considering a separation value, the 15 tests use an overlap of the form:
//    <tt>overlap = r1 + r2 - D</tt>,
// where r1 and r2 are the half-projections of the two boxes on the current direction
// and D is the projected distance between the box centers. If there's no overlap
// (overlap <= 0) in any direction, then the boxes do not intersect. Otherwise, we
// keep track of the direction of minimum overlap.
//
// If considering a separation > 0, we simply use an overlap of the form:
//    <tt>overlap = r1 + r2 - D + separation</tt>
// and use the exact same checks.
int box_intersects_box(const real3& hdims1,
                       const real3& hdims2,
                       const real3& pos,
                       const quaternion& rot,
                       real separation,
                       real3& dir) {
    Mat33 R(rot);
    Mat33 Rabs = Abs(R);
    real minOverlap = C_REAL_MAX;

    // Test the axes of the 1st box.
    for (uint i = 0; i < 3; i++) {
        real r2 = Rabs(i, 0) * hdims2[0] + Rabs(i, 1) * hdims2[1] + Rabs(i, 2) * hdims2[2];
        real overlap = hdims1[i] + r2 - abs(pos[i]) + separation;

        if (overlap <= 0)
            return 0;

        if (overlap < minOverlap) {
            dir = real3(0);
            dir[i] = 1;
            minOverlap = overlap;
        }
    }

    // Test the axes of the 2nd box.
    for (uint i = 0; i < 3; i++) {
        real r1 = Rabs(0, i) * hdims1[0] + Rabs(1, i) * hdims1[1] + Rabs(2, i) * hdims1[2];
        real overlap = r1 + hdims2[i] - abs(R(0, i) * pos[0] + R(1, i) * pos[1] + R(2, i) * pos[2]) + separation;

        if (overlap <= 0)
            return 0;

        if (overlap < minOverlap) {
            dir = real3(R(0, i), R(1, i), R(2, i));
            minOverlap = overlap;
        }
    }

    // Test the planes that are orthogonal (the cross-product) to pairs of axes of the two boxes.
    for (uint x1 = 0, y1 = 1, z1 = 2; x1 < 3; y1 = z1, z1 = x1++) {
        for (uint x2 = 0, y2 = 1, z2 = 2; x2 < 3; y2 = z2, z2 = x2++) {
            real3 crossProd;

            crossProd[x1] = 0;
            crossProd[y1] = -R(z1, x2);
            crossProd[z1] = R(y1, x2);

            real lengthSqr = Dot(crossProd);

            if (lengthSqr > 1e-6) {
                real r1 = hdims1[y1] * Rabs(z1, x2) + hdims1[z1] * Rabs(y1, x2);
                real r2 = hdims2[y2] * Rabs(x1, z2) + hdims2[z2] * Rabs(x1, y2);
                real overlap = r1 + r2 - abs(pos[z1] * R(y1, x2) - pos[y1] * R(z1, x2)) + separation;

                if (overlap <= 0)
                    return 0;

                real ooLen = 1 / Sqrt(lengthSqr);

                overlap *= ooLen;
                if (overlap < minOverlap) {
                    dir = crossProd * ooLen;
                    minOverlap = overlap;
                }
            }
        }
    }

    // If minOverlap is larger than the specified separation, the boxes actually penetrate.
    // Otherwise, they are separated (but not by more that 'separation').
    return (minOverlap >= separation) ? -1 : +1;
}

// This function returns an integer indicating whether or not a box with dimensions hdims intersects (or is close
// enough to) a triangle given by its vertices v0, v1, v2.  The check is performed in the box frame and it is assumed
// that the triangle vertices are expressed in the box frame.
//
// The return value is -1 if the box and triangle overlap, +1 if they are within a distance of 'separation' from each
// other, and 0 if they are "far" from each other.
//
// This check is performed by testing 13 possible separating planes between the box and triangle (see Ericson).
int box_intersects_triangle(const real3& hdims,
                            const real3& v0,
                            const real3& v1,
                            const real3& v2,
                            real separation) {
    real3 hdimsS = hdims + separation;
    real minOverlap = C_REAL_MAX;
    real overlap;

    // (1) Test the 3 axes corresponding to the box face normals
    for (int i = 0; i < 3; i++) {
        overlap = Max(v0[i], v1[i], v2[i]) + hdimsS[i];
        if (overlap <= 0)
            return 0;
        if (overlap < minOverlap) {
            minOverlap = overlap;
        }
        overlap = hdimsS[i] - Min(v0[i], v1[i], v2[i]);
        if (overlap <= 0)
            return 0;
        if (overlap < minOverlap) {
            minOverlap = overlap;
        }
    }

    // Triangle normal vector
    real3 n = triangle_normal(v0, v1, v2);

    // (2) Test the axis corresponding to the triangle normal
    real r = hdimsS[0] * Abs(n[0]) + hdimsS[1] * Abs(n[1]) + hdimsS[2] * Abs(n[2]);
    real d = Dot(n, v0);
    overlap = r - Abs(d);
    if (overlap <= 0)
        return 0;
    if (overlap < minOverlap) {
        minOverlap = overlap;
    }

    // Calculate triangle edge vectors
    real3 f[] = {
        v1 - v0,  // edge V0V1
        v2 - v1,  // edge V1V2
        v0 - v2   // edge V2V0
    };

    // (3) Test axis corresponding to cross products of triangle edges and box directions
    //       --------|----------0----*-----|-----------*-------
    //              -r               m    +r           M
    for (int i = 0; i < 3; i++) {
        // For each of the 3 box directions (k=0,1,2), calculate:
        // (a) cross product of current triangle edge f[i] with the box direction (a_k)
        // (b) projection radius of box onto the cross product (r_k)
        // (c) projection of each triangle vertex onto the cross product:
        //     p0_k = V0 . a_k , p1_k = V1 . a_k , p2_k = V2 . a_k
        // (d) projection interval of triangle onto the cross product:
        //     [m_k , M_k] = [min(p0_k, p1_k, p2_k) , max(p0_k, p1_k, p2_k)]
        // (e) overlap = 2 * r_k - Max(m_k+r_k, -M_k-r_k)

        // box direction u0 = [1,0,0]
        real3 a0(0, -f[i].z, +f[i].y);  // a0 = [1, 0, 0] x f[i]
        real a0_len = Length(a0);
        if (a0_len > 1e-10) {
            real r0 = hdimsS.y * Abs(f[i].z) + hdimsS.z * Abs(f[i].y);
            real p0_0 = -v0.y * f[i].z + v0.z * f[i].y;
            real p1_0 = -v1.y * f[i].z + v1.z * f[i].y;
            real p2_0 = -v2.y * f[i].z + v2.z * f[i].y;
            real m0 = Min(p0_0, p1_0, p2_0);
            real M0 = Max(p0_0, p1_0, p2_0);
            overlap = 2 * r0 - Max(m0 + r0, -M0 - r0);
            if (overlap <= 0)
                return 0;
            overlap /= a0_len;
            if (overlap < minOverlap) {
                minOverlap = overlap;
            }
        }

        // box direction u1 = [0,1,0]
        real3 a1(+f[i].z, 0, -f[i].z);  // a1 = [0, 1, 0] x f[i]
        real a1_len = Length(a1);
        if (a1_len > 1e-10) {
            real r1 = hdimsS.x * Abs(f[i].z) + hdimsS.z * Abs(f[i].x);
            real p0_1 = v0.x * f[i].z - v0.z * f[i].x;
            real p1_1 = v1.x * f[i].z - v1.z * f[i].x;
            real p2_1 = v2.x * f[i].z - v2.z * f[i].x;
            real m1 = Min(p0_1, p1_1, p2_1);
            real M1 = Max(p0_1, p1_1, p2_1);
            overlap = 2 * r1 - Max(m1 + r1, -M1 - r1);
            if (overlap <= 0)
                return 0;
            overlap /= a1_len;
            if (overlap < minOverlap) {
                minOverlap = overlap;
            }
        }

        // box direction u2 = [0,0,1]
        real3 a2(-f[i].y, +f[i].x, 0);  // a2 = [0, 0, 1] x f[i]
        real a2_len = Length(a2);
        if (a2_len > 1e-10) {
            real r2 = hdimsS.x * Abs(f[i].y) + hdimsS.y * Abs(f[i].x);
            real p0_2 = -v0.x * f[i].y + v0.y * f[i].x;
            real p1_2 = -v1.x * f[i].y + v1.y * f[i].x;
            real p2_2 = -v2.x * f[i].y + v2.y * f[i].x;
            real m2 = Min(p0_2, p1_2, p2_2);
            real M2 = Max(p0_2, p1_2, p2_2);
            overlap = 2 * r2 - Max(m2 + r2, -M2 - r2);
            if (overlap <= 0)
                return 0;
            overlap /= a2_len;
            if (overlap < minOverlap) {
                minOverlap = overlap;
            }
        }
    }

    // If minOverlap is larger than the specified separation, the box and triangle actually penetrate.
    // Otherwise, they are separated (but not by more that 'separation').
    return (minOverlap >= separation) ? -1 : +1;
}

}  // end namespace ch_utils
}  // end namespace collision
}  // end namespace chrono
