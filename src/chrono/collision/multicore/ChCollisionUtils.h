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
// Miscellaneous utility functions for the Chrono multicore collision detection
// system.
// These are low-level functions used in broadphase, for MPR narrowphase, and in
// specialized pair-wise collision detection for PRIMS narrowphase.
//
// =============================================================================

#pragma once

#include <cassert>

#include "chrono/multicore_math/ChMulticoreMath.h"
#include "chrono/multicore_math/matrix.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/multicore/ChConvexShape.h"

namespace chrono {

/// @addtogroup collision_mc
/// @{

/// Utilities for Chrono multicore collision detection
namespace ch_utils {

// =============================================================================

/// @name Hashing functions
/// @{

/// Convert a position into bin coordinates ("lower" corner).
template <class T>
inline vec3 HashMin(const T& A, const real3& inv_bin_size_vec) {
    vec3 temp;
    temp.x = (int)Floor(A.x * inv_bin_size_vec.x);
    temp.y = (int)Floor(A.y * inv_bin_size_vec.y);
    temp.z = (int)Floor(A.z * inv_bin_size_vec.z);
    return temp;
}

/// Convert a position into bin coordinates ("upper" corner).
template <class T>
inline vec3 HashMax(const T& A, const real3& inv_bin_size_vec) {
    vec3 temp;
    temp.x = (int)Ceil(A.x * inv_bin_size_vec.x) - 1;
    temp.y = (int)Ceil(A.y * inv_bin_size_vec.y) - 1;
    temp.z = (int)Ceil(A.z * inv_bin_size_vec.z) - 1;
    return temp;
}

/// Convert bin coordinates into a unique bin index value.
inline uint Hash_Index(const vec3& A, const vec3& bins_per_axis) {
    return ((A.z * bins_per_axis.y) * bins_per_axis.x) + (A.y * bins_per_axis.x) + A.x;
}

/// Decode a bin index into its associated bin coordinates.
inline vec3 Hash_Decode(uint hash, const vec3& bins_per_axis) {
    vec3 decoded_hash;
    decoded_hash.x = hash % (bins_per_axis.x * bins_per_axis.y) % bins_per_axis.x;
    decoded_hash.y = (hash % (bins_per_axis.x * bins_per_axis.y)) / bins_per_axis.x;
    decoded_hash.z = hash / (bins_per_axis.x * bins_per_axis.y);
    return decoded_hash;
}

/// @}

// =============================================================================

/// @name AABB collision functions
/// @{

/// Check if two bodies interact using their collision family data.
inline bool collide(const short2& fam_data_A, const short2& fam_data_B) {
    // Return true only if the bit corresponding to family of B is set in the mask
    // of A and vice-versa.
    return (fam_data_A.y & fam_data_B.x) && (fam_data_B.y & fam_data_A.x);
}

/// Check if two AABBs overlap using their min/max corners.
inline bool overlap(const real3& Amin, const real3& Amax, const real3& Bmin, const real3& Bmax) {
    // Return true only if the two AABBs overlap in all 3 directions.
    return (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
           (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
}

inline bool current_bin(const real3& Amin,
                        const real3& Amax,
                        const real3& Bmin,
                        const real3& Bmax,
                        const real3& inv_bin_size_vec,
                        const vec3& bins_per_axis,
                        uint bin) {
    real3 min_p = Max(Amin, Bmin);

    if (Hash_Index(HashMin(min_p, inv_bin_size_vec), bins_per_axis) == bin) {
        return true;
    }
    return false;
}

/// @}

// =============================================================================

/// @name Utility functions for broadphase
/// @{

/// Compute broadphase grid resolution, given the total number of AABBs, the grid dimension (diagonal), and the desired
/// density.
ChApi vec3 Compute_Grid_Resolution(uint num_aabb, const real3& d, real k);

/// Function to Count AABB-Bin intersections.
ChApi void f_Count_AABB_BIN_Intersection(const uint index,
                                         const real3& inv_bin_size,
                                         const std::vector<real3>& aabb_min,
                                         const std::vector<real3>& aabb_max,
                                         std::vector<uint>& bins_intersected);

/// Function to Store AABB-Bin Intersections.
ChApi void f_Store_AABB_BIN_Intersection(const uint index,
                                         const vec3& bins_per_axis,
                                         const real3& inv_bin_size,
                                         const std::vector<real3>& aabb_min_data,
                                         const std::vector<real3>& aabb_max_data,
                                         const std::vector<uint>& bins_intersected,
                                         std::vector<uint>& bin_number,
                                         std::vector<uint>& aabb_number);

/// Function to count AABB-AABB intersection.
ChApi void f_Count_AABB_AABB_Intersection(const uint index,
                                          const real3 inv_bin_size_vec,
                                          const vec3 bins_per_axis,
                                          const std::vector<real3>& aabb_min_data,
                                          const std::vector<real3>& aabb_max_data,
                                          const std::vector<uint>& bin_number,
                                          const std::vector<uint>& aabb_number,
                                          const std::vector<uint>& bin_start_index,
                                          const std::vector<short2>& fam_data,
                                          const std::vector<char>& body_active,
                                          const std::vector<char>& body_collide,
                                          const std::vector<uint>& body_id,
                                          std::vector<uint>& num_contact);

/// Function to store AABB-AABB intersections.
ChApi void f_Store_AABB_AABB_Intersection(const uint index,
                                          const real3 inv_bin_size_vec,
                                          const vec3 bins_per_axis,
                                          const std::vector<real3>& aabb_min_data,
                                          const std::vector<real3>& aabb_max_data,
                                          const std::vector<uint>& bin_number,
                                          const std::vector<uint>& aabb_number,
                                          const std::vector<uint>& bin_start_index,
                                          const std::vector<uint>& num_contact,
                                          const std::vector<short2>& fam_data,
                                          const std::vector<char>& body_active,
                                          const std::vector<char>& body_collide,
                                          const std::vector<uint>& body_id,
                                          std::vector<long long>& potential_contacts);

/// @}

// =============================================================================
/*

/// @name Utility functions for two-level broadphase
/// @{

ChApi void f_TL_Count_Leaves(const uint index,
                             const real density,
                             const real3& bin_size,
                             const std::vector<uint>& bin_start_index,
                             std::vector<uint>& leaves_per_bin);

/// Count the number of AABB leaf intersections for each bin.
ChApi void f_TL_Count_AABB_Leaf_Intersection(const uint index,
                                             const real density,
                                             const real3& bin_size,
                                             const vec3& bins_per_axis,
                                             const std::vector<uint>& bin_start_index,
                                             const std::vector<uint>& bin_number,
                                             const std::vector<uint>& shape_number,
                                             const std::vector<real3>& aabb_min,
                                             const std::vector<real3>& aabb_max,
                                             std::vector<uint>& leaves_intersected);

/// Store the AABB leaf intersections for each bin.
ChApi void f_TL_Write_AABB_Leaf_Intersection(const uint& index,
                                             const real density,
                                             const real3& bin_size,
                                             const vec3& bin_resolution,
                                             const std::vector<uint>& bin_start_index,
                                             const std::vector<uint>& bin_number,
                                             const std::vector<uint>& bin_shape_number,
                                             const std::vector<real3>& aabb_min,
                                             const std::vector<real3>& aabb_max,
                                             const std::vector<uint>& leaves_intersected,
                                             const std::vector<uint>& leaves_per_bin,
                                             std::vector<uint>& leaf_number,
                                             std::vector<uint>& leaf_shape_number);

/// @}

*/

// =============================================================================

/// @name Utility functions for MPR narrowphase
/// @{

/// Support point for a sphere (for GJK and MPR).
inline real3 GetSupportPoint_Sphere(const real radius, const real3& n) {
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
    return result;
}

/// Support point for a cylinder (for GJK and MPR).
/// Cylinder assumed to be along Z axis with origin at center.
inline real3 GetSupportPoint_Cylinder(const real3& B, const real3& n) {
    const real& radius = B.x;
    const real& hheight = B.z;

    // If direction along cone axis, support point arbitrary on base circumference
    real s = Sqrt(n.x * n.x + n.y * n.y);
    if (s < 1e-9)
        return real3(radius, 0, n.z < 0.0 ? -hheight : hheight);

    // Support point on base circumference
    return real3(n.x * radius / s, n.y * radius / s, n.z < 0.0 ? -hheight : hheight);
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
/// Cone assumed to be along Z axis with origing at base center.
inline real3 GetSupportPoint_Cone(const real3& B, const real3& n) {
    const real& radius = B.x;
    const real& height = B.y;

    // If direction close to cone axis, support point at apex
    real sinAngle = (radius / Sqrt(radius * radius + height * height));
    if (n.z > Length(n) * sinAngle)
        return real3(0, 0, height);

    // If direction along cone axis downwards, support point at center of base
    real s = Sqrt(n.x * n.x + n.y * n.y);
    if (s < 1e-9)
        return real3(0, 0, 0);

    // Support point on base circumference
    return real3(n.x * radius / s, n.y * radius / s, 0);
}

/// Support point for a line segment (for GJK and MPR).
inline real3 GetSupportPoint_Seg(const real B, const real3& n) {
    real3 result = real3(0);
    result.x = Sign(n.x) * B;

    return result;
}

/// Support point for a capsule (for GJK and MPR).
/// Capsule assumed to be along Z axis with origin at center.
inline real3 GetSupportPoint_Capsule(const real2& B, const real3& n) {
    return GetSupportPoint_Seg(B.y, n) + GetSupportPoint_Sphere(B.x, n);
}

/// Support point for a disk (for GJK and MPR).
inline real3 GetSupportPoint_Disk(const real B, const real3& n) {
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
/// Rounded cylinder assumed to be along Z axis with origin at center.
inline real3 GetSupportPoint_RoundedCylinder(const real4& B, const real3& n) {
    return GetSupportPoint_Cylinder(real3(B.x, B.y, B.z), n) + GetSupportPoint_Sphere(B.w, n);
}

/// Support point for a cylindrical shell (for GJK and MPR).
/// Cylindrical shell assumed to be along Z axis with origin at center.
inline real3 GetSupportPoint_CylindricalShell(const real3& B, const real3& n) {
    return GetSupportPoint_Cylinder(real3(B.x, B.y, B.z), n);
}

/// Support point for a generic convex shape (for GJK and MPR).
inline real3 GetSupportPoint_Convex(const int size, const real3* convex_data, const real3& n) {
    real max_dot_p = -C_REAL_MAX;
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
inline real3 GetSupportPoint_Tetrahedron(const uvec4& indices, const real3* nodes, const real3& n) {
    real max_dot_p = -C_REAL_MAX;
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

inline real3 GetCenter_Tetrahedron(const uvec4& indices, const real3* nodes) {
    real3 tet = nodes[indices.x] + nodes[indices.y] + nodes[indices.z] + nodes[indices.w];
    return tet / real(4.0);
}

ChApi real3 SupportVertNoMargin(const chrono::ConvexBase* Shape, const real3& nv, const real& envelope);

ChApi real3 LocalSupportVert(const chrono::ConvexBase* Shape, const real3& n, const real& envelope);

ChApi real3 TransformSupportVert(const chrono::ConvexBase* Shape, const real3& n, const real& envelope);

/// This utility function takes the location 'P' and snaps it to the closest point on the triangular face with given
/// vertices (A, B, and C). Additionally, it also returns the barycentric coordinates of the result point. See
/// snap_to_triangle for additional details.
ChApi bool snap_to_triangle_bary(const real3& A,
                                 const real3& B,
                                 const real3& C,
                                 const real3& P,
                                 real3& res,
                                 real3& barycentric);

/// Given a contact point P and a tetrahedron T compute the closest triangle to that point.
ChApi void FindTriIndex(const real3& P, const uvec4& T, const real3* pos_node, int& face, real3& cb);

/// @}

// =============================================================================

/// @name Utility functions for PRIMS narrowphase.
/// @{

/// This utility function returns the normal to the triangular face defined by the vertices A, B, and C. The face is
/// assumed to be non-degenerate. Note that order of vertices is important!
inline real3 triangle_normal(const real3& A, const real3& B, const real3& C) {
    real3 v1 = B - A;
    real3 v2 = C - A;
    real3 n = Cross(v1, v2);
    real len = Length(n);

    return n / len;
}

/// This utility function takes the location 'P' and snaps it to the closest point on the triangular face with given
/// vertices (A, B, and C). The result is returned in 'res'. Both 'P' and 'res' are assumed to be specified in the same
/// frame as the face vertices. This function returns 'true' if the result is on an edge of this face and 'false' if the
/// result is inside the triangle. Code from Ericson, "Real-time collision detection", 2005, pp. 141
ChApi bool snap_to_triangle(const real3& A, const real3& B, const real3& C, const real3& P, real3& res);

/// This utility function returns a boolean indicating whether or not the specified 'loc' is inside the given triangle
/// (with vertices A, B, C and outwards 'normal'). Locations on a triangle edge are considered to be inside.
ChApi bool point_in_triangle(const real3& A, const real3& B, const real3& C, const real3& loc);

/// This utility function snaps the specified location to a point on a cylinder with given radius and half-length. The
/// in/out location is assumed to be specified in the frame of the cylinder (in this frame the cylinder is assumed to be
/// centered at the origin and aligned with the Z axis).  The return code indicates the feature of the cylinder that
/// caused snapping.
///   - code = 0 indicates and interior point
///   - code = 1 indicates snapping to one of the cylinder caps
///   - code = 2 indicates snapping to the cylinder side
///   - code = 3 indicates snapping to one of the cylinder edges
inline uint snap_to_cylinder(const real& rad, const real& hlen, real3& loc) {
    uint code = 0;

    if (loc.z > hlen) {
        code |= 1;
        loc.z = hlen;
    } else if (loc.z < -hlen) {
        code |= 1;
        loc.z = -hlen;
    }

    real d2 = loc.x * loc.x + loc.y * loc.y;

    if (d2 > rad * rad) {
        code |= 2;
        real d = Sqrt(d2);
        loc.x *= (rad / d);
        loc.y *= (rad / d);
    }

    return code;
}

/// This utility function returns a code that indicates the closest feature of a box in the specified direction. The
/// direction 'dir' is assumed to be given in the frame of the box. The return code encodes the box axes that define the
/// closest feature:
///   - first bit (least significant) corresponds to x-axis
///   - second bit corresponds to y-axis
///   - third bit corresponds to z-axis
///
/// Therefore:
///   - code = 0 indicates a degenerate direction (within a threshold)
///   - code = 1 or code = 2 or code = 4  indicates a face
///   - code = 3 or code = 5 or code = 6  indicates an edge
///   - code = 7 indicates a corner
inline uint box_closest_feature(const real3& dir, const real3& hdims) {
    real threshold = 0.01;  // corresponds to about 0.57 degrees
    return ((Abs(dir.x) > threshold) << 0) | ((Abs(dir.y) > threshold) << 1) | ((Abs(dir.z) > threshold) << 2);
}

/// This utility function snaps the specified location to a point on a box with given half-dimensions. The in/out
/// location is assumed to be specified in the frame of the box (which is therefore assumed to be an AABB centered at
/// the origin).  The return code indicates the box axes that caused snapping.
///   - first bit (least significant) corresponds to x-axis
///   - second bit corresponds to y-axis
///   - third bit corresponds to z-axis
///
/// Therefore:
///   - code = 0 indicates an interior point
///   - code = 1 or code = 2 or code = 4  indicates snapping to a face
///   - code = 3 or code = 5 or code = 6  indicates snapping to an edge
///   - code = 7 indicates snapping to a corner
inline uint snap_to_box(const real3& hdims, real3& loc) {
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

/// This utility function snaps the location 'pt_to_snap' to a location on the face of this box identified by the
/// specified 'code' and 'pt_on_box'. The resulting location is returned.
/// See box_closest_feature for definition of 'code.
inline real3 snap_to_box_face(const real3& hdims, const real3& pt_on_box, uint code, const real3& pt_to_snap) {
    uint side = code >> 1;
    real3 A;
    real3 B;
    real3 C;
    real3 result = pt_to_snap;

    if (side == 0) {
        result.x = pt_on_box.x;
        ClampValue(result.y, -hdims.y, hdims.y);
        ClampValue(result.z, -hdims.z, hdims.z);
    } else if (side == 1) {
        result.y = pt_on_box.y;
        ClampValue(result.x, -hdims.x, hdims.x);
        ClampValue(result.z, -hdims.z, hdims.z);
    } else {
        result.z = pt_on_box.z;
        ClampValue(result.x, -hdims.x, hdims.x);
        ClampValue(result.y, -hdims.y, hdims.y);
    }

    return result;
}

/// This utility function snaps the location 'pt_to_snap' to a location on the edge of this box identified by the
/// specified 'code' and 'pt_on_box'. The resulting location is returned.
/// See box_closest_feature for definition of 'code.
inline real3 snap_to_box_edge(const real3& hdims, const real3& pt_on_box, uint code, const real3& pt_to_snap) {
    uint axis = (~code & 7) >> 1;

    real3 pt1 = pt_on_box;
    real3 pt2 = pt_on_box;

    if (axis == 0) {
        pt1.x = -hdims.x;
        pt2.x = hdims.x;
    } else if (axis == 1) {
        pt1.y = -hdims.y;
        pt2.y = hdims.y;
    } else {
        pt1.z = -hdims.z;
        pt2.z = hdims.z;
    }

    real3 edge = pt2 - pt1;
    real t = Dot((pt_to_snap - pt1), edge) / (Length(edge) * Length(edge));

    return pt1 + t * edge;
}

/// This utility function fill out 'corners' with the corners of the box edge specified by 'code' and 'pt_on_edge'.
/// See box_closest_feature for definition of 'code.
inline void get_edge_corners(const real3& pt_on_edge, uint code, real3* corners) {
    code = ((~code & 7) >> 1);

    corners[0] = pt_on_edge;
    corners[1] = pt_on_edge;

    if (code == 0) {
        corners[1].x = -corners[1].x;
    } else if (code == 1) {
        corners[1].y = -corners[1].y;
    } else {
        corners[1].z = -corners[1].z;
    }
}

/// This utility function fill out 'corners' with the corners of the face specified by 'code' and 'pt_on_face'.
/// See box_closest_feature for definition of 'code.
inline void get_face_corners(const real3& pt_on_face, uint code, real3* corners) {
    code = code >> 1;

    corners[0] = pt_on_face;
    corners[1] = pt_on_face;

    if (code == 0) {
        corners[1].y = -corners[1].y;
        corners[2] = corners[1];
        corners[2].z = -corners[1].z;
        corners[3] = pt_on_face;
        corners[3].z = corners[2].z;
    } else if (code == 1) {
        corners[1].z = -corners[1].z;
        corners[2] = corners[1];
        corners[2].x = -corners[1].x;
        corners[3] = pt_on_face;
        corners[3].x = corners[2].x;
    } else {
        corners[1].x = -corners[1].x;
        corners[2] = corners[1];
        corners[2].y = -corners[1].y;
        corners[3] = pt_on_face;
        corners[3].y = corners[2].y;
    }
}

/// This utility function returns the box face normal (expressed in the box local frame) for the face specified by
/// 'code' and 'pt_on_face'.
/// See box_closest_feature for definition of 'code.
inline real3 get_face_normal(const real3& pt_on_face, uint code) {
    if (code == 1) {
        real nx = (pt_on_face.x > 0 ? +1 : -1);
        return real3(nx, 0, 0);
    }
    if (code == 2) {
        real ny = (pt_on_face.y > 0 ? +1 : -1);
        return real3(0, ny, 0);
    }
    if (code == 4) {
        real nz = (pt_on_face.z > 0 ? +1 : -1);
        return real3(0, 0, nz);
    }
    assert(false);
    return real3(0, 0, 0);
}

/// This utility function returns true if 'point' is no farther than 'separation' from the face of this box identified
/// by 'pt_on_face' and 'code'; and returns false otherwise. If the point is close enough, its projection onto the box
/// face is returned in 'result' and the height of the point relative to the box face is returned in 'dist' (with a
/// negative value indicating penetration, i.e. the point inside the box). 'normal' is set to the box face (pointing
/// outwards). See box_closest_feature for definition of 'code.
ChApi bool point_vs_face(const real3& hdims,
                         const real3& pt_on_face,
                         uint code,
                         const real3& point,
                         const real& separation,
                         real3& result,
                         real3& normal,
                         real& dist);

/// This utility function calculates the closest points between the box edge identified by 'pt_on_edge' and 'code' and a
/// line segment between 'pt1' and 'pt2'. It returns true if the closest points are within the extent of the edge and
/// the segment, respecively, and false otherwise. The closest points are returned in 'locE' and 'locS', respectively.
/// This function uses the parametric solution of the two support lines to solve for the closest points, simplified as
/// we work in the local frame of the box.
/// See box_closest_feature for definition of 'code.
ChApi bool segment_vs_edge(const real3& hdims,
                           const real3& pt_on_edge,
                           uint code,
                           const real3& pt1,
                           const real3& pt2,
                           real3& locE,
                           real3& locS);

/// This utility function returns the corner of a box of given dimensions that is farthest in the direction 'dir', which
/// is assumed to be given in the frame of the box.
inline real3 box_farthest_corner(const real3& hdims, const real3& dir) {
    real3 corner;
    corner.x = (dir.x < 0) ? hdims.x : -hdims.x;
    corner.y = (dir.y < 0) ? hdims.y : -hdims.y;
    corner.z = (dir.z < 0) ? hdims.z : -hdims.z;
    return corner;
}

/// This utility function returns the corner of a box of given dimensions that is closest in the direction 'dir', which
/// is assumed to be given in the frame of the box.
inline real3 box_closest_corner(const real3& hdims, const real3& dir) {
    real3 corner;
    corner.x = (dir.x > 0) ? hdims.x : -hdims.x;
    corner.y = (dir.y > 0) ? hdims.y : -hdims.y;
    corner.z = (dir.z > 0) ? hdims.z : -hdims.z;
    return corner;
}

/// This function returns an integer indicating whether or not a box1 with dimensions hdims1 intersects (or is close
/// enough to) a second box with dimensions hdims2. The check is performed in the local frame of box1. The transform
/// from the other box is given through 'pos' and 'rot'.
///
/// The return value is -1 if the two boxes overlap, +1 if they are within a distance of 'separation' from each other,
/// and 0 if they are "far" from each other. If returning -1 or +1, 'dir' contains the direction of smallest
/// intersection (or closest separation).
///
/// This check is performed by testing 15 possible separating planes between the two boxes (Gottschalk, Lin, Manocha -
/// Siggraph96).
///
/// If not considering a separation value, the 15 tests use an overlap of the form:
///    <tt>overlap = r1 + r2 - D</tt>,
/// where r1 and r2 are the half-projections of the two boxes on the current direction and D is the projected distance
/// between the box centers. If there's no overlap (overlap <= 0) in any direction, then the boxes do not intersect.
/// Otherwise, we keep track of the direction of minimum overlap.
///
/// If considering a separation > 0, we simply use an overlap of the form:
///    <tt>overlap = r1 + r2 - D + separation</tt>
/// and use the exact same checks.
ChApi int box_intersects_box(const real3& hdims1,
                             const real3& hdims2,
                             const real3& pos,
                             const quaternion& rot,
                             real separation,
                             real3& dir);

/// This function returns an integer indicating whether or not a box with dimensions hdims intersects (or is close
/// enough to) a triangle given by its vertices v0, v1, v2.  The check is performed in the box frame and it is assumed
/// that the triangle vertices are expressed in the box frame.
///
/// The return value is -1 if the box and triangle overlap, +1 if they are within a distance of 'separation' from each
/// other, and 0 if they are "far" from each other.
///
/// This check is performed by testing 13 possible separating planes between the box and triangle (see Ericson).
ChApi int box_intersects_triangle(const real3& hdims,
                                  const real3& v0,
                                  const real3& v1,
                                  const real3& v2,
                                  real separation);

/// @}

}  // end namespace ch_utils

/// @} collision_mc

}  // end namespace chrono
