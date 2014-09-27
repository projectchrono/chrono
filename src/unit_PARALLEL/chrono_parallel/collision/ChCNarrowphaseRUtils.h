// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#ifndef CHC_NARROWPHASE_R_UTILS_H
#define CHC_NARROWPHASE_R_UTILS_H


namespace chrono {
namespace collision {


// ----------------------------------------------------------------------------
// This utility function returns the normal to the triangular face defined by
// the vertices A, B, and C. The face is assumed to be non-degenerate.
// Note that order of vertices is important!
__host__ __device__
real3 face_normal(const real3& A, const real3& B, const real3& C)
{
  real3 v1 = B - A;
  real3 v2 = C - A;
  real3 n = cross(v1, v2);
  real len = length(n);

  return n / len;
}

// ----------------------------------------------------------------------------
// This utility function takes the location 'P' and snaps it to the closest
// point on the triangular face with given vertices (A, B, and C). The result
// is returned in 'res'. Both 'P' and 'res' are assumed to be specified in
// the same frame as the face vertices. This function returns 'true' if the
// result is on an edge of this face and 'false' if the result is inside the
// triangle.
// Code from Ericson, "Real-time collision detection", 2005, pp. 141
__host__ __device__
bool snap_to_face(const real3& A, const real3& B, const real3& C,
                  const real3& P, real3& res)
{
  real3 AB = B - A;
  real3 AC = C - A;

  // Check if P in vertex region outside A
  real3 AP = P - A;
  real d1 = dot(AB, AP);
  real d2 = dot(AC, AP);
  if (d1 <= 0 && d2 <= 0) {
    res = A;               // barycentric coordinates (1,0,0)
    return true;
  }

  // Check if P in vertex region outside B
  real3 BP = P - B;
  real d3 = dot(AB, BP);
  real d4 = dot(AC, BP);
  if (d3 >= 0 && d4 <= d3) {
    res = B;                // barycentric coordinates (0,1,0)
    return true;
  }

  // Check if P in edge region of AB
  real vc = d1*d4 - d3*d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    // Return projection of P onto AB
    real v = d1 / (d1 - d3);
    res = A + v * AB;      // barycentric coordinates (1-v,v,0)
    return true;
  }

  // Check if P in vertex region outside C
  real3 CP = P - C;
  real d5 = dot(AB, CP);
  real d6 = dot(AC, CP);
  if (d6 >= 0 && d5 <= d6) {
    res = C;                // barycentric coordinates (0,0,1)
    return true;
  }

  // Check if P in edge region of AC
  real vb = d5*d2 - d1*d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    // Return projection of P onto AC
    real w = d2 / (d2 - d6);
    res = A + w * AC;       // barycentric coordinates (1-w,0,w)
    return true;
  }

  // Check if P in edge region of BC
  real va = d3*d6 - d5*d4;
  if (va <= 0 && (d4-d3) >= 0 && (d5-d6) >= 0) {
    // Return projection of P onto BC
    real w = (d4-d3) / ((d4-d3) + (d5-d6));
    res = B + w * (C-B);    // barycentric coordinates (0,1-w,w)
    return true;
  }

  // P inside face region. Return projection of P onto face
  // barycentric coordinates (u,v,w)
  real denom = 1 / (va + vb + vc);
  real v = vb * denom;
  real w = vc * denom;
  res = A + v * AB + w * AC;   // = u*A + v*B + w*C  where  (u = 1 - v - w)
  return false;
}

// ----------------------------------------------------------------------------
// This utility function snaps the specified location to a point on a cylinder
// with given radius and half-length. The in/out location is assumed to be
// specified in the frame of the cylinder (in this frame the cylinder is assumed
// to be centered at the origin and aligned with the Y axis).  The return code
// indicates the feature of the cylinder that caused snapping.
//   code = 0 indicates and interior point
//   code = 1 indicates snapping to one of the cylinder caps
//   code = 2 indicates snapping to the cylinder side
//   code = 3 indicates snapping to one of the cylinder edges
__host__ __device__
uint snap_to_cylinder(const real& rad, const real& hlen, real3& loc)
{
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
    real d = sqrt(d2);
    loc.x *= (rad/d);
    loc.z *= (rad/d);
  }

  return code;
}

// ----------------------------------------------------------------------------
// This utility function snaps the specified location to a point on a box with
// given half-dimensions. The in/out location is assumed to be specified in
// the frame of the box (which is therefore assumed to be an AABB centered at
// the origin).  The return code indicates the box axes that caused snapping.
//   - first bit (least significant) corresponds to x-axis
//   - second bit corresponds to y-axis
//   - third bit corresponds to z-axis
// Therefore:
//   code = 0 indicates an interior point
//   code = 1 or code = 2 or code = 4  indicates snapping to a face
//   code = 3 or code = 5 or code = 6  indicates snapping to an edge
//   code = 7 indicates snapping to a corner
__host__ __device__
uint snap_to_box(const real3& hdims, real3& loc)
{
  uint code = 0;

  if (fabs(loc.x) > hdims.x) {
    code |= 1;
    loc.x = (loc.x > 0) ? hdims.x : -hdims.x;
  }
  if (fabs(loc.y) > hdims.y) {
    code |= 2;
    loc.y = (loc.y > 0) ? hdims.y : -hdims.y;
  }
  if (fabs(loc.z) > hdims.z) {
    code |= 4;
    loc.z = (loc.z > 0) ? hdims.z : -hdims.z;
  }

  return code;
}


// ----------------------------------------------------------------------------
// These utility functions return the corner of a box of given dimensions that
// if farthest and closest in the direction 'dir', respectively. The direction
// 'dir' is assumed to be given in the frame of the box.
__host__ __device__
real3 box_farthest_corner(const real3& hdims, const real3& dir)
{
  real3 corner;
  corner.x = (dir.x < 0) ? hdims.x : -hdims.x;
  corner.y = (dir.y < 0) ? hdims.y : -hdims.y;
  corner.z = (dir.z < 0) ? hdims.z : -hdims.z;
  return corner;
}

__host__ __device__
real3 box_closest_corner(const real3& hdims, const real3& dir)
{
  real3 corner;
  corner.x = (dir.x > 0) ? hdims.x : -hdims.x;
  corner.y = (dir.y > 0) ? hdims.y : -hdims.y;
  corner.z = (dir.z > 0) ? hdims.z : -hdims.z;
  return corner;
}


// ----------------------------------------------------------------------------
// This utility function returns a code that indicates the closest feature of
// a box in the specified direction. The direction 'dir' is assumed to be
// given in the frame of the box. The return code encodes the box axes that
// define the closest feature:
//   - first bit (least significant) corresponds to x-axis
//   - second bit corresponds to y-axis
//   - third bit corresponds to z-axis
// Therefore:
//   code = 0 indicates a degenerate direction (within a threshold)
//   code = 1 or code = 2 or code = 4  indicates a face
//   code = 3 or code = 5 or code = 6  indicates an edge
//   code = 7 indicates a corner
__host__ __device__
uint box_closest_feature(const real3& dir)
{
  const real threshold = 0.01;

  return ((fabs(dir.x) > threshold) << 0) |
         ((fabs(dir.y) > threshold) << 1) |
         ((fabs(dir.z) > threshold) << 2);
}


// ----------------------------------------------------------------------------
// This function returns a boolean indicating whether or not a box1 with
// dimensions hdims1 intersects a second box with the dimensions hdims2.
// The check is performed in the local frame of box1. The transform from the
// other box is given through 'pos' and 'rot'. If an intersection exists, the
// direction of smallest intersection is returned in 'dir'.
//
// This check is performed by testing 15 possible separating planes between the
// two boxes (Gottschalk, Lin, Manocha - Siggraph96).
__host__ __device__
bool box_intersects_box(const real3& hdims1, const real3& hdims2,
                        const real3& pos, const real4& rot,
                        real3& dir)
{
  M33 R = AMat(rot);
  M33 Rabs = AbsMat(R);
  real minOverlap = FLT_MAX;
  real overlap;
  real r1, r2;

  // 1. Test the axes of box1 (3 cases)
  // x-axis
  r2 = Rabs.U.x * hdims2.x + Rabs.V.x * hdims2.y + Rabs.W.x * hdims2.z;
  overlap = hdims1.x + r2 - fabs(pos.x);
  if (overlap <= 0) return false;
  if (overlap < minOverlap) {
    dir = R3(1, 0, 0);
    minOverlap = overlap;
  }
  // y-axis
  r2 = Rabs.U.y * hdims2.x + Rabs.V.y * hdims2.y + Rabs.W.y * hdims2.z;
  overlap = hdims1.y + r2 - fabs(pos.y);
  if (overlap <= 0) return false;
  if (overlap < minOverlap) {
    dir = R3(0, 1, 0);
    minOverlap = overlap;
  } 
  // z-axis
  r2 = Rabs.U.z * hdims2.x + Rabs.V.z * hdims2.y + Rabs.W.z * hdims2.z;
  overlap = hdims1.z + r2 - fabs(pos.z);
  if (overlap <= 0) return false;
  if (overlap < minOverlap) {
    dir = R3(0, 0, 1);
    minOverlap = overlap;
  } 

  // 2. Test the axes of box2 (3 cases)
  // x-axis
  r1 = dot(Rabs.U, hdims1);
  overlap = r1 + hdims2.x - fabs(dot(R.U, pos));
  if (overlap <= 0) return false;
  if (overlap < minOverlap) {
    dir = R.U;
    minOverlap = overlap;
  }
  // y-axis
  r1 = dot(Rabs.V, hdims1);
  overlap = r1 + hdims2.y - fabs(dot(R.V, pos));
  if (overlap <= 0) return false;
  if (overlap < minOverlap) {
    dir = R.V;
    minOverlap = overlap;
  }
  // z-axis
  r1 = dot(Rabs.W, hdims1);
  overlap = r1 + hdims2.z - fabs(dot(R.W, pos));
  if (overlap <= 0) return false;
  if (overlap < minOverlap) {
    dir = R.W;
    minOverlap = overlap;
  }

  // 3. Test the planes that are orthogonal (the cross-product) to pairs of axes
  // of the two boxes (9 cases)


  //// TODO

  return false;
}


// ----------------------------------------------------------------------------


} // end namespace collision
} // end namespace chrono


#endif

