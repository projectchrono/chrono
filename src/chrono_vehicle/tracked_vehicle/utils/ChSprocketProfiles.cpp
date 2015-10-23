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
// Utility functions for generating parameterized 2D sprocket gear profiles.
// These are used as 2D collision shapes (against similar profiles for track
// shoe bodies). They are composed of a number of sub-paths of type ChLineArc
// or ChLineSegment, defined in the X-Y plane.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChVector.h"
#include "chrono/core/ChSpmatrix.h"  //// HACK to force using the exported ChMatrix<double> specialization from there!!!!
#include "chrono/geometry/ChCLineSegment.h"
#include "chrono/geometry/ChCLineArc.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChSprocketProfiles.h"

namespace chrono {
namespace vehicle {

ChSharedPtr<geometry::ChLinePath> ChCircularProfile(int num_teeth, double R_T, double R_C, double R) {
    ChSharedPtr<geometry::ChLinePath> path(new geometry::ChLinePath);

    double beta = CH_C_2PI / num_teeth;
    double sbeta = std::sin(beta / 2);
    double cbeta = std::cos(beta / 2);
    double y = (R_T * R_T + R_C * R_C - R * R) / (2 * R_C);
    double x = std::sqrt(R_T * R_T - y * y);
    double gamma = std::asin(x / R);

    for (int i = 0; i < num_teeth; ++i) {
        double alpha = -i * beta;
        ChVector<> p0(0, R_C, 0);
        ChVector<> p1(-R_T * sbeta, R_T * cbeta, 0);
        ChVector<> p2(-x, y, 0);
        ChVector<> p3(x, y, 0);
        ChVector<> p4(R_T * sbeta, R_T * cbeta, 0);
        ChQuaternion<> quat;
        quat.Q_from_AngZ(alpha);
        ChMatrix33<> rot(quat);
        p0 = rot * p0;
        p1 = rot * p1;
        p2 = rot * p2;
        p3 = rot * p3;
        p4 = rot * p4;
        geometry::ChLineSegment seg1(p1, p2);
        double angle1 = alpha + 1.5 * CH_C_PI - gamma;
        double angle2 = alpha + 1.5 * CH_C_PI + gamma;
        geometry::ChLineArc arc(ChCoordsys<>(p0), R, angle1, angle2, true);
        geometry::ChLineSegment seg2(p3, p4);
        path->AddSubLine(seg1);
        path->AddSubLine(arc);
        path->AddSubLine(seg2);
    }

    return path;
}

}  // end namespace vehicle
}  // end namespace chrono
