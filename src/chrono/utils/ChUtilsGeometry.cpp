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

#include <cstdlib>
#include <cmath>

#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/core/ChMatrix33.h"

namespace chrono {
namespace utils {

#define EPS 1e-20
#define EPS_TRIDEGEN 1e-10

// Calculate the line segment PaPb that is the shortest route between
// two lines P1P2 and P3P4. Calculate also the values of mua and mub where
//    Pa = P1 + mua (P2 - P1)
//    Pb = P3 + mub (P4 - P3)
// Return false if no solution exists.
bool LineLineIntersect(const ChVector3d& p1,
                       const ChVector3d& p2,
                       const ChVector3d& p3,
                       const ChVector3d& p4,
                       ChVector3d* pa,
                       ChVector3d* pb,
                       double* mua,
                       double* mub) {
    ChVector3d p13, p43, p21;
    double d1343, d4321, d1321, d4343, d2121;
    double numer, denom;

    p13.x() = p1.x() - p3.x();
    p13.y() = p1.y() - p3.y();
    p13.z() = p1.z() - p3.z();
    p43.x() = p4.x() - p3.x();
    p43.y() = p4.y() - p3.y();
    p43.z() = p4.z() - p3.z();
    if (fabs(p43.x()) < EPS && fabs(p43.y()) < EPS && fabs(p43.z()) < EPS)
        return false;
    p21.x() = p2.x() - p1.x();
    p21.y() = p2.y() - p1.y();
    p21.z() = p2.z() - p1.z();
    if (fabs(p21.x()) < EPS && fabs(p21.y()) < EPS && fabs(p21.z()) < EPS)
        return false;

    d1343 = p13.x() * p43.x() + p13.y() * p43.y() + p13.z() * p43.z();
    d4321 = p43.x() * p21.x() + p43.y() * p21.y() + p43.z() * p21.z();
    d1321 = p13.x() * p21.x() + p13.y() * p21.y() + p13.z() * p21.z();
    d4343 = p43.x() * p43.x() + p43.y() * p43.y() + p43.z() * p43.z();
    d2121 = p21.x() * p21.x() + p21.y() * p21.y() + p21.z() * p21.z();

    denom = d2121 * d4343 - d4321 * d4321;
    if (fabs(denom) < EPS)
        return false;
    numer = d1343 * d4321 - d1321 * d4343;

    *mua = numer / denom;
    *mub = (d1343 + d4321 * (*mua)) / d4343;

    pa->x() = p1.x() + *mua * p21.x();
    pa->y() = p1.y() + *mua * p21.y();
    pa->z() = p1.z() + *mua * p21.z();
    pb->x() = p3.x() + *mub * p43.x();
    pb->y() = p3.y() + *mub * p43.y();
    pb->z() = p3.z() + *mub * p43.z();

    return true;
}

// Calculate distance between a point B and the segment (A1,A2).
// Returns the distance from B to the line and sets the line parameter 'u' such that u=0 indicates that B projects into
// A1 and u=1 indicates that B projects into A2.  If 0 <= u <= 1, in_segment is set to 'true'.
double PointLineDistance(const ChVector3d& B, const ChVector3d& A1, const ChVector3d& A2, double& u, bool& in_segment) {
    u = -1;
    in_segment = false;
    double mdist = 10e34;

    ChVector3d vseg = Vsub(A2, A1);
    ChVector3d vdir = Vnorm(vseg);
    ChVector3d vray = Vsub(B, A1);

    mdist = Vlength(Vcross(vray, vdir));
    u = Vdot(vray, vdir) / Vlength(vseg);

    if (u >= 0 && u <= 1)
        in_segment = true;

    return mdist;
}

// Calculate distance of a point from a triangle surface.
// Also computes if projection is inside the triangle.
double PointTriangleDistance(const ChVector3d& B,
                             const ChVector3d& A1,
                             const ChVector3d& A2,
                             const ChVector3d& A3,
                             double& u,
                             double& v,
                             bool& in_triangle,
                             ChVector3d& Bprojected) {
    // defaults
    in_triangle = false;
    u = v = -1;
    double mdistance = 10e22;

    ChVector3d Dx, Dy, Dz, T1, T1p;

    Dx = Vsub(A2, A1);
    Dz = Vsub(A3, A1);
    Dy = Vcross(Dz, Dx);

    double dylen = Vlength(Dy);

    if (fabs(dylen) < EPS_TRIDEGEN)  // degenerate triangle
        return mdistance;

    Dy = Vmul(Dy, 1.0 / dylen);

    ChMatrix33<> mA(Dx, Dy, Dz);

    // invert triangle coordinate matrix -if singular matrix, was degenerate triangle-.
    if (std::abs(mA.determinant()) < 0.000001)
        return mdistance;

    ChMatrix33<> mAi = mA.inverse();
    T1 = mAi * (B - A1);
    T1p = T1;
    T1p.y() = 0;
    u = T1.x();
    v = T1.z();
    mdistance = -T1.y();
    if (u >= 0 && v >= 0 && v <= 1 - u) {
        in_triangle = true;
        Bprojected = A1 + mA * T1p;
    }

    return mdistance;
}

bool DegenerateTriangle(const ChVector3d& Dx, const ChVector3d& Dy) {
    ChVector3d vcr;
    vcr = Vcross(Dx, Dy);
    if (fabs(vcr.x()) < EPS_TRIDEGEN && fabs(vcr.y()) < EPS_TRIDEGEN && fabs(vcr.z()) < EPS_TRIDEGEN)
        return true;
    return false;
}

bool DegenerateTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) {
    return DegenerateTriangle(v2 - v1, v3 - v1);
}

}  // end namespace utils
}  // end namespace chrono
