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

#ifndef CH_COLLISION_UTILS_BULLET_H
#define CH_COLLISION_UTILS_BULLET_H

#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/collision/bullet/LinearMath/btVector3.h"
#include "chrono/collision/bullet/LinearMath/btMinMax.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_bullet
/// @{

/// Utilities for Bullet-based collision detection
namespace bt_utils {

/// Project point onto line.
ChApi btVector3 ProjectPointOnLine(const btVector3& lP,  ///< point on line
                                   const btVector3& lD,  ///< line direction (unit vector)
                                   const btVector3& P    ///< point
);

/// Calculate distance from point to line
ChApi btScalar DistancePointToLine(const btVector3& lP,  ///< point on line
                                   const btVector3& lD,  ///< line direction (unit vector)
                                   const btVector3& P    ///< point
);

/// Snap the specified location to a point on a box with given half-dimensions.
/// The in/out location is assumed to be specified in the frame of the box (which is therefore assumed to be an AABB
/// centered at the origin).  The return code indicates the box axes that caused snapping:
///   - first bit (least significant) corresponds to x-axis
///   - second bit corresponds to y-axis
///   - third bit corresponds to z-axis
///
/// Therefore:
///   code = 0 indicates an interior point
///   code = 1 or code = 2 or code = 4  indicates snapping to a face
///   code = 3 or code = 5 or code = 6  indicates snapping to an edge
///   code = 7 indicates snapping to a corner
ChApi int SnapPointToBox(const btVector3& hdims,  ///< box half-dimensions
                         btVector3& loc           ///< point (in/out)
);

/// Check if given point is inside box (point expressed in box frame).
ChApi bool PointInsideBox(const btVector3& hdims,  ///< box half-dimensions
                          const btVector3& loc     ///< point

);

/// Find the closest box face to the given point (expressed in box frame).
/// Returns +1, +2, +3 (for a "positive" face in x, y, z, respectively) or -1, -2, -3 (for a "negative" face).
ChApi int FindClosestBoxFace(const btVector3& hdims,  ///< box half-dimensions
                             const btVector3& loc     ///< point
);

// Utility function for intersecting a box with a line segment.
// It is assumed that the box is centered at the origin and the segment is expressed in the box frame.
// The function returns false if the segment does not intersect the box.
ChApi bool IntersectSegmentBox(const btVector3& hdims,  ///< box half-dimensions
                               const btVector3& c,      ///< segment center point
                               const btVector3& a,      ///< segment direction (unit vector)
                               const btScalar hlen,     ///< segment half-length
                               const btScalar tol,      ///< tolerance for parallelism test
                               btScalar& tMin,          ///< segment parameter of first intersection point
                               btScalar& tMax           ///< segment parameter of second intersection point
);

/// Utility function to intersect a line with a plane
/// Plane equation: pN.X = pN.pP
/// Line equation:  X = lP + t * lD
/// Solution:       t = pN.(pP-lP) / pN.lD
ChApi bool IntersectLinePlane(const btVector3& lP,  ///< point on line
                              const btVector3& lD,  ///< line direction (unit vector)
                              const btVector3& pP,  ///< point on plane
                              const btVector3& pN,  ///< plane normal (unit vector)
                              const btScalar tol,   ///< tolerance for orthogonality test
                              btScalar& t           ///< line parameter of intersection point
);

/// Utility function to intersect a segment with a cylinder.
/// Segment assumed to be parameterized as sP = sC + t * sD, with -sH <= t <= sH.
/// Cylinder given by its center cC, axis direction cD, halh-lengh cH, and radius cR.
/// Assume |sD| = |cD| = 1.
/// (1) Find tMin and tMax where the segment supporting line intersects cylindrical surface by finding
///     points on segment at a distance cR from the cylinder axis line.
/// (2) Clamp result to cylinder end-caps.
/// (3) Clamp result to segment length.
ChApi bool IntersectSegmentCylinder(const btVector3& sC,  ///< segment center point
                                    const btVector3& sD,  ///< segment direction (unit vector)
                                    const btScalar sH,    ///< segment half-length
                                    const btVector3& cC,  ///< cylinder axis center
                                    const btVector3& cD,  ///< cylinder axis direction (unit vector)
                                    const btScalar cH,    ///< cylinder axis half-length (cylinder halh-height)
                                    const btScalar cR,    ///< cylinder radius
                                    const btScalar tol,   ///< tolerance for parallelism test
                                    btScalar& tMin,       ///< segment parameter of first intersection point
                                    btScalar& tMax        ///< segment parameter of second intersection point
);

/// Wrapper for using and exporting the Bullet implementation of the convex hull library.
class ChApi ChConvexHullLibraryWrapper {
  public:
    ChConvexHullLibraryWrapper();
    void ComputeHull(const std::vector<ChVector<> >& points, geometry::ChTriangleMeshConnected& vshape);
};

}  // namespace bt_utils

/// @} collision_bullet

}  // namespace collision
}  // namespace chrono

#endif
