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
// Authors: Alessandro Tasora, Radu Serban, Dario Fusai
// =============================================================================

#include <cstdlib>
#include <cmath>

#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/core/ChMatrix33.h"

namespace chrono {
namespace utils {

static constexpr double EPS = 1e-20;
static constexpr double EPS_TRIDEGEN = 1e-10;

bool ClosestLinesPoints(const ChVector3d& p1, const ChVector3d& p2, const ChVector3d& p3, const ChVector3d& p4, ChVector3d& pa, ChVector3d& pb, double& ua, double& ub) {
    ChVector3d p12 = p2 - p1;
    if (std::abs(p12.x()) < EPS && std::abs(p12.y()) < EPS && std::abs(p12.z()) < EPS)
        return false;

    ChVector3d p34 = p4 - p3;
    if (std::abs(p34.x()) < EPS && std::abs(p34.y()) < EPS && std::abs(p34.z()) < EPS)
        return false;

    ChVector3d p31 = p1 - p3;
    double d3134 = Vdot(p31, p34);
    double d3412 = Vdot(p34, p12);
    double d3112 = Vdot(p31, p12);
    double d3434 = Vdot(p34, p34);
    double d1212 = Vdot(p12, p12);

    double denom = d1212 * d3434 - d3412 * d3412;
    if (std::abs(denom) < EPS)
        return false;
    double numer = d3134 * d3412 - d3112 * d3434;

    ua = numer / denom;
    ub = (d3134 + d3412 * ua) / d3434;
    pa = p1 + ua * p12;
    pb = p3 + ub * p34;

    return true;
}

double PointLineDistance(const ChVector3d& B, const ChVector3d& A1, const ChVector3d& A2, double& u, bool& in_segment) {
    in_segment = false;

    ChVector3d line_seg = A2 - A1;
    ChVector3d line_dir = line_seg.GetNormalized();
    ChVector3d A1B = B - A1;

    double proj = Vdot(A1B, line_dir);
    double dist = Vlength(A1B - proj * line_dir);

    u = proj / Vlength(line_seg);
    if (u >= 0 && u <= 1)
        in_segment = true;

    return dist;
}

double PointTrianglePlaneDistance(const ChVector3d& B,    //
                                  const ChVector3d& A1,   //
                                  const ChVector3d& A2,   //
                                  const ChVector3d& A3,   //
                                  double& u,              //
                                  double& v,              //
                                  bool& in_triangle,      //
                                  ChVector3d& Bprojected  //
) {
    // Defaults
    in_triangle = false;
    u = -1.0;
    v = -1.0;

    ChVector3d dx = A2 - A1;
    ChVector3d dy = A3 - A1;
    ChVector3d normal = Vcross(dx, dy);  // not yet normalized
    double n_len = normal.Length();

    // Check degenerate triangle
    if (n_len < EPS_TRIDEGEN) {
        return std::numeric_limits<double>::max();
    }

    normal = normal / n_len;                 // normalize now
    double distance = Vdot(B - A1, normal);  // signed distance
    Bprojected = B - normal * distance;

    // Compute barycentric coordinates
    ChVector3d wp = Bprojected - A1;
    double d00 = Vdot(dx, dx);
    double d01 = Vdot(dx, dy);
    double d11 = Vdot(dy, dy);
    double d20 = Vdot(wp, dx);
    double d21 = Vdot(wp, dy);
    double denom = d00 * d11 - d01 * d01;

    u = (d11 * d20 - d01 * d21) / denom;
    v = (d00 * d21 - d01 * d20) / denom;
    in_triangle = (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0);

    return distance;
}

bool IsTriangleDegenerate(const ChVector3d& Dx, const ChVector3d& Dy) {
    ChVector3d vcr;
    vcr = Vcross(Dx, Dy);
    if (std::abs(vcr.x()) < EPS_TRIDEGEN && std::abs(vcr.y()) < EPS_TRIDEGEN && std::abs(vcr.z()) < EPS_TRIDEGEN)
        return true;
    return false;
}

bool IsTriangleDegenerate(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) {
    return IsTriangleDegenerate(v2 - v1, v3 - v1);
}

ChVector3d ClosestTrianglePointToPoint(const ChVector3d& p, const ChVector3d& a, const ChVector3d& b, const ChVector3d& c) {
    // Check if P in vertex region outside A
    ChVector3d ab = b - a;
    ChVector3d ac = c - a;
    ChVector3d ap = p - a;
    double d1 = Vdot(ab, ap);
    double d2 = Vdot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return a;  // barycentric coordinates (1,0,0)
    }

    // Check if P in vertex region outside B
    ChVector3d bp = p - b;
    double d3 = Vdot(ab, bp);
    double d4 = Vdot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return b;  // barycentric coordinates (0,1,0)
    }

    // Check if P in edge region of AB, if so return projection of P onto AB
    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        double v = d1 / (d1 - d3);
        return a + v * ab;  // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    ChVector3d cp = p - c;
    double d5 = Vdot(ab, cp);
    double d6 = Vdot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return c;  // barycentric coordinates (0,0,1)
    }

    // Check if P in edge region of AC, if so return projection of P onto AC
    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        double w = d2 / (d2 - d6);
        return a + w * ac;  // barycentric coordinates (1-w,0,w)
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);  // barycentric coordinates (0,1-w,w)
    }

    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    return a + ab * v + ac * w;  // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
}

}  // end namespace utils
}  // end namespace chrono
