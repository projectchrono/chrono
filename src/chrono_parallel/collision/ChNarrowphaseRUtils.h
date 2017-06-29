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
// Authors: Radu Serban
// =============================================================================
//
// Header file for ChCNarrowphaseRUtils.
// This file defines various low-level utility functions used in specialized
// pair-wise collision detection (e.g., finding the closest point on a shape to
// a specified point).
//
// =============================================================================

#pragma once

#include "chrono_parallel/math/matrix.h"

namespace chrono {
namespace collision {

/// @addtogroup parallel_collision
/// @{

/// This utility function returns the normal to the triangular face defined by
/// the vertices A, B, and C. The face is assumed to be non-degenerate.
/// Note that order of vertices is important!
real3 face_normal(const real3& A, const real3& B, const real3& C) {
    real3 v1 = B - A;
    real3 v2 = C - A;
    real3 n = Cross(v1, v2);
    real len = Length(n);

    return n / len;
}

/// This utility function takes the location 'P' and snaps it to the closest
/// point on the triangular face with given vertices (A, B, and C). The result
/// is returned in 'res'. Both 'P' and 'res' are assumed to be specified in
/// the same frame as the face vertices. This function returns 'true' if the
/// result is on an edge of this face and 'false' if the result is inside the
/// triangle.
/// Code from Ericson, "Real-time collision detection", 2005, pp. 141
bool snap_to_face(const real3& A, const real3& B, const real3& C, const real3& P, real3& res) {
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

/// This utility function snaps the specified location to a point on a cylinder
/// with given radius and half-length. The in/out location is assumed to be
/// specified in the frame of the cylinder (in this frame the cylinder is assumed
/// to be centered at the origin and aligned with the Y axis).  The return code
/// indicates the feature of the cylinder that caused snapping.
///   code = 0 indicates and interior point
///   code = 1 indicates snapping to one of the cylinder caps
///   code = 2 indicates snapping to the cylinder side
///   code = 3 indicates snapping to one of the cylinder edges
uint snap_to_cylinder(const real& rad, const real& hlen, real3& loc) {
    uint code = 0;

    if (loc.y > hlen) {
        code |= 1;
        loc.y = hlen;
    } else if (loc.y < -hlen) {
        code |= 1;
        loc.y = -hlen;
    }

    real d2 = loc.x * loc.x + loc.z * loc.z;

    if (d2 > rad * rad) {
        code |= 2;
        real d = Sqrt(d2);
        loc.x *= (rad / d);
        loc.z *= (rad / d);
    }

    return code;
}

/// This utility function snaps the specified location to a point on a box with
/// given half-dimensions. The in/out location is assumed to be specified in
/// the frame of the box (which is therefore assumed to be an AABB centered at
/// the origin).  The return code indicates the box axes that caused snapping.
///   - first bit (least significant) corresponds to x-axis
///   - second bit corresponds to y-axis
///   - third bit corresponds to z-axis
///
/// Therefore:
///   code = 0 indicates an interior point
///   code = 1 or code = 2 or code = 4  indicates snapping to a face
///   code = 3 or code = 5 or code = 6  indicates snapping to an edge
///   code = 7 indicates snapping to a corner
uint snap_to_box(const real3& hdims, real3& loc) {
    uint code = 0;

    if (Abs(loc.x) > hdims.x) {
        code |= 1;
        loc.x = (loc.x > 0) ? hdims.x : -hdims.x;
    }
    if (Abs(loc.y) > hdims.y) {
        code |= 2;
        loc.y = (loc.y > 0) ? hdims.y : -hdims.y;
    }
    if (Abs(loc.z) > hdims.z) {
        code |= 4;
        loc.z = (loc.z > 0) ? hdims.z : -hdims.z;
    }

    return code;
}

/// This utility function returns the corner of a box of given dimensions that
/// if farthest in the direction 'dir', which is assumed to be given in the frame of the box.
real3 box_farthest_corner(const real3& hdims, const real3& dir) {
    real3 corner;
    corner.x = (dir.x < 0) ? hdims.x : -hdims.x;
    corner.y = (dir.y < 0) ? hdims.y : -hdims.y;
    corner.z = (dir.z < 0) ? hdims.z : -hdims.z;
    return corner;
}

/// This utility function returns the corner of a box of given dimensions that
/// if closest in the direction 'dir', which is assumed to be given in the frame of the box.
real3 box_closest_corner(const real3& hdims, const real3& dir) {
    real3 corner;
    corner.x = (dir.x > 0) ? hdims.x : -hdims.x;
    corner.y = (dir.y > 0) ? hdims.y : -hdims.y;
    corner.z = (dir.z > 0) ? hdims.z : -hdims.z;
    return corner;
}

/// This utility function returns a code that indicates the closest feature of
/// a box in the specified direction. The direction 'dir' is assumed to be
/// given in the frame of the box. The return code encodes the box axes that
/// define the closest feature:
///   - first bit (least significant) corresponds to x-axis
///   - second bit corresponds to y-axis
///   - third bit corresponds to z-axis
///
/// Therefore:
///   code = 0 indicates a degenerate direction (within a threshold)
///   code = 1 or code = 2 or code = 4  indicates a face
///   code = 3 or code = 5 or code = 6  indicates an edge
///   code = 7 indicates a corner
uint box_closest_feature(const real3& dir) {
    const real threshold = 0.01;

    return ((Abs(dir.x) > threshold) << 0) | ((Abs(dir.y) > threshold) << 1) | ((Abs(dir.z) > threshold) << 2);
}

/// This function returns a boolean indicating whether or not a box1 with
/// dimensions hdims1 intersects a second box with the dimensions hdims2.
/// The check is performed in the local frame of box1. The transform from the
/// other box is given through 'pos' and 'rot'. If an intersection exists, the
/// direction of smallest intersection is returned in 'dir'.
///
/// This check is performed by testing 15 possible separating planes between the
/// two boxes (Gottschalk, Lin, Manocha - Siggraph96).
bool box_intersects_box(const real3& hdims1, const real3& hdims2, const real3& pos, const quaternion& rot, real3& dir) {
    Mat33 R(rot);
    Mat33 Rabs = Abs(R);
    real minOverlap = FLT_MAX;
    real overlap;
    real r1, r2;

    // 1. Test the axes of box1 (3 cases)
    // x-axis
    r2 = Rabs[0] * hdims2.x + Rabs[4] * hdims2.y + Rabs[8] * hdims2.z;
    overlap = hdims1.x + r2 - Abs(pos.x);
    if (overlap <= 0)
        return false;
    if (overlap < minOverlap) {
        dir = real3(1, 0, 0);
        minOverlap = overlap;
    }
    // y-axis
    r2 = Rabs[1] * hdims2.x + Rabs[5] * hdims2.y + Rabs[9] * hdims2.z;
    overlap = hdims1.y + r2 - Abs(pos.y);
    if (overlap <= 0)
        return false;
    if (overlap < minOverlap) {
        dir = real3(0, 1, 0);
        minOverlap = overlap;
    }
    // z-axis
    r2 = Rabs[2] * hdims2.x + Rabs[6] * hdims2.y + Rabs[10] * hdims2.z;
    overlap = hdims1.z + r2 - Abs(pos.z);
    if (overlap <= 0)
        return false;
    if (overlap < minOverlap) {
        dir = real3(0, 0, 1);
        minOverlap = overlap;
    }

    // 2. Test the axes of box2 (3 cases)
    // x-axis
    r1 = Dot(Rabs.col(0), hdims1);
    overlap = r1 + hdims2.x - Abs(Dot(R.col(0), pos));
    if (overlap <= 0)
        return false;
    if (overlap < minOverlap) {
        dir = R.col(0);
        minOverlap = overlap;
    }
    // y-axis
    r1 = Dot(Rabs.col(1), hdims1);
    overlap = r1 + hdims2.y - Abs(Dot(R.col(1), pos));
    if (overlap <= 0)
        return false;
    if (overlap < minOverlap) {
        dir = R.col(1);
        minOverlap = overlap;
    }
    // z-axis
    r1 = Dot(Rabs.col(2), hdims1);
    overlap = r1 + hdims2.z - Abs(Dot(R.col(2), pos));
    if (overlap <= 0)
        return false;
    if (overlap < minOverlap) {
        dir = R.col(2);
        minOverlap = overlap;
    }

    // 3. Test the planes that are orthogonal (the cross-product) to pairs of axes
    // of the two boxes (9 cases)

    //// TODO

    return false;
}


/// @} parallel_colision

} // end namespace collision
} // end namespace chrono
