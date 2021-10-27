// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_COLLISION_UTILS_H
#define CH_COLLISION_UTILS_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"

namespace chrono {
namespace collision {

/// @addtogroup chrono_collision
/// @{

/// Collision detection utility functions
namespace utils {

/// Calculate the line segment PaPb that is the shortest route between
/// two lines P1P2 and P3P4. Calculate also the values of mua and mub where
///    Pa = P1 + mua (P2 - P1)
///    Pb = P3 + mub (P4 - P3)
/// Return false if no solution exists.
ChApi bool LineLineIntersect(const ChVector<>& p1,
                             const ChVector<>& p2,
                             const ChVector<>& p3,
                             const ChVector<>& p4,
                             ChVector<>* pa,
                             ChVector<>* pb,
                             double* mua,
                             double* mub);

/// Calculate distance between a point p and a line identified
/// with segment dA,dB. Returns distance. Also, the mu value reference
/// tells if the nearest projection of point on line falls into segment (for mu 0...1)
ChApi double PointLineDistance(const ChVector<>& p,
                               const ChVector<>& dA,
                               const ChVector<>& dB,
                               double& mu,
                               bool& is_insegment);

/// Calculate distance of a point from a triangle surface.
/// Also computes if projection is inside the triangle.
/// If is_into = true, Bprojected is also computed.
/// Returns distance (positive if 'out' side, out is where points A1 A2 A3 can be read in clockwise fashion)
ChApi double PointTriangleDistance(const ChVector<>& B,
                                   const ChVector<>& A1,
                                   const ChVector<>& A2,
                                   const ChVector<>& A3,
                                   double& mu,
                                   double& mv,
                                   bool& is_into,
                                   ChVector<>& Bprojected);

/// Check if the triangle defined by the two given vectors is degenerate.
ChApi bool DegenerateTriangle(const ChVector<>& Dx, const ChVector<>& Dy);

/// Check if the triangle defined by the three given vertices is degenerate.
ChApi bool DegenerateTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3);

}  // end namespace utils

/// @} chrono_collision

}  // end namespace collision
}  // end namespace chrono

#endif
