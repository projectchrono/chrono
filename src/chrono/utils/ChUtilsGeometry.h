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
// Authors: Radu Serban, Hammad Mazhar, Arman Pazouki, Dario Fusai
// =============================================================================
//
// Utility functions for various geometrical calculations.
//
// =============================================================================

#ifndef CH_UTILS_GEOMETRY_H
#define CH_UTILS_GEOMETRY_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Calculate the line segment (Pa, Pb) that is the shortest route between two lines (P1, P2) and (P3, P4).
/// Calculate also the values of ua and ub where
///    Pa = P1 + ua * (P2 - P1)
///    Pb = P3 + ub * (P4 - P3)
/// Return false if no solution exists.
ChApi bool LineLineIntersect(const ChVector3d& p1, const ChVector3d& p2, const ChVector3d& p3, const ChVector3d& p4, ChVector3d& pa, ChVector3d& pb, double& ua, double& ub);

/// Calculate and return the distance between a point B and the segment (A1, A2).
/// Provided line parameter 'u' is set such that u=0 indicates that B projects into A1 and u=1 indicates that B projects into A2.
/// If 0 <= u <= 1, 'in_segment' is set to true.
ChApi double PointLineDistance(const ChVector3d& B, const ChVector3d& A1, const ChVector3d& A2, double& u, bool& in_segment);

/// Calculate and return distance of a point from a triangle surface plane.
/// Returned distance value is positive if given point lies on triangle 'out' side (out is where points A1 A2 A3 can be read in clockwise fashion).
/// Additionally:
/// - compute projected point in barycentric coordinates u, v
/// - check if point projection lies inside the triangle ('in_triangle' set to true)
/// - compute point projection ('Bprojected') if in_triangle = true.
ChApi double PointTrianglePlaneDistance(const ChVector3d& B, const ChVector3d& A1, const ChVector3d& A2, const ChVector3d& A3, double& u, double& v, bool& in_triangle, ChVector3d& Bprojected);

/// Check if the triangle defined by the two given vectors is degenerate.
ChApi bool IsTriangleDegenerate(const ChVector3d& Dx, const ChVector3d& Dy);

/// Check if the triangle defined by the three given vertices is degenerate.
ChApi bool IsTriangleDegenerate(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3);

/// Compute and return the closest triangle point to given point p (Ericson algorithm).
ChApi ChVector3d ClosestTrianglePointToPoint(const ChVector3d& p, const ChVector3d& a, const ChVector3d& b, const ChVector3d& c);

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
