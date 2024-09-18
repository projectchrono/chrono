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
// Authors: Radu Serban, Hammad Mazhar, Arman Pazouki
// =============================================================================
//
// Utility functions for various geometrical calculations.
//
// =============================================================================

#ifndef CH_UTILS_GEOMETRY_H
#define CH_UTILS_GEOMETRY_H

#include <cmath>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"

#include "chrono/collision/ChCollisionModel.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Calculate the line segment PaPb that is the shortest route between two lines P1P2 and P3P4.
/// Calculate also the values of mua and mub where
///    Pa = P1 + mua (P2 - P1)
///    Pb = P3 + mub (P4 - P3)
/// Return false if no solution exists.
ChApi bool LineLineIntersect(const ChVector3d& p1,
                             const ChVector3d& p2,
                             const ChVector3d& p3,
                             const ChVector3d& p4,
                             ChVector3d* pa,
                             ChVector3d* pb,
                             double* mua,
                             double* mub);

/// Calculate distance between a point B and the segment (A1,A2).
/// Returns the distance from B to the line and sets the line parameter 'u' such that u=0 indicates that B projects into
/// A1 and u=1 indicates that B projects into A2. If 0 <= u <= 1, in_segment is set to 'true'.
ChApi double PointLineDistance(const ChVector3d& B,
                               const ChVector3d& A1,
                               const ChVector3d& A2,
                               double& u,
                               bool& in_segment);

/// Calculate distance of a point from a triangle surface.
/// Also computes if projection is inside the triangle. If is_into = true, Bprojected is also computed.
/// Returns distance (positive if 'out' side, out is where points A1 A2 A3 can be read in clockwise fashion).
ChApi double PointTriangleDistance(const ChVector3d& B,
                                   const ChVector3d& A1,
                                   const ChVector3d& A2,
                                   const ChVector3d& A3,
                                   double& u,
                                   double& v,
                                   bool& in_triangle,
                                   ChVector3d& Bprojected);

/// Check if the triangle defined by the two given vectors is degenerate.
ChApi bool DegenerateTriangle(const ChVector3d& Dx, const ChVector3d& Dy);

/// Check if the triangle defined by the three given vertices is degenerate.
ChApi bool DegenerateTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3);

/*

// Volume calculations

inline double CalcBiSphereVolume(double radius, double c_dist) {
    double delta = 2 * radius - c_dist;
    double cos_theta = (radius - 0.5 * delta) / radius;
    return (4.0 / 3.0) * CH_PI * radius * radius * radius * (1 + cos_theta);
}

inline double CalcTorusVolume(double radius, double thickness) {
    return 2 * CH_PI * CH_PI * thickness * thickness * radius;
}

// Gyration calculations

inline ChMatrix33<> CalcBiSphereGyration(double radius, double c_dist) {
    // TODO: simple implementation for now

    double delta = 2 * radius - c_dist;
    double cos_theta = (radius - 0.5 * delta) / radius;
    double z_prim = radius - 0.5 * delta;

    double comp1 = 0.4 * radius * radius * (1 + cos_theta);
    double comp2 = -0.2 * radius * radius * (1. / 3. * (-cos_theta * cos_theta * cos_theta - 1) + (1 + cos_theta));
    double comp3 = 2. / 3. * z_prim * z_prim * (1 + cos_theta);
    double comp4 = 0.5 * radius * z_prim * std::sqrt(1 - cos_theta * cos_theta);
    double numerator = 2 * (comp1 + comp2 + comp3 + comp4);
    double denominator = 4. / 3. * (1 + cos_theta);
    double Jxx = numerator / denominator;
    double Jyy = 0.6 * radius * radius * (1. / 3. * (-cos_theta * cos_theta * cos_theta - 1) + (1 + cos_theta)) /
                 (1 + cos_theta);

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = Jxx;
    J(1, 1) = Jyy;
    J(2, 2) = Jxx;

    return J;
}

inline ChMatrix33<> CalcTorusGyration(double radius, double thickness) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (5.0 / 8.0) * (thickness * thickness) + (1.0 / 2.0) * (radius * radius);
    J(1, 1) = (3.0 / 4.0) * (thickness * thickness) + (radius * radius);
    J(2, 2) = (5.0 / 8.0) * (thickness * thickness) + (1.0 / 2.0) * (radius * radius);

    return J;
}

*/

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
