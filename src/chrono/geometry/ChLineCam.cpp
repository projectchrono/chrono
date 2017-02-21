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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/geometry/ChLineCam.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineCam)

ChLineCam::ChLineCam() {
    Set_complexity(10);
    this->closed = true;
    type = CAM_TYPE_SLIDEFOLLOWER;
    Rb = 1.0;
    Rr = 0.0;
    p = 1.8;
    d = 2;
    b0 = CH_C_PI / 6.0;
    center = VNULL;
    e = 0;
    s = Rb;
    law = std::make_shared<ChFunction_Const>(0);  // default law = no follower motion
    negative = false;
    internal = false;
}

ChLineCam::ChLineCam(const ChLineCam& source) : ChLine(source) {
    type = source.type;
    Rb = source.Rb;
    Rr = source.Rr;
    p = source.p;
    d = source.d;
    b0 = source.b0;
    center = source.center;
    e = source.e;
    s = source.s;
    law = source.law;
    negative = source.negative;
    internal = source.internal;
}

void ChLineCam::Set_Rb(double mrb) {
    Rb = mrb;
    if (e > 0.9 * Rb)
        e = 0.9 * Rb;
    if (e < -0.9 * Rb)
        e = -0.9 * Rb;
}

void ChLineCam::Set_rotating_follower(double mp, double md, double mb0) {
    p = mp;
    d = md;
    b0 = mb0;
    Rb = sqrt(p * p + d * d - 2 * p * d * cos(b0));
}

void ChLineCam::Set_flat_oscillate(double me, double md, double mb0) {
    e = me;
    d = md;
    b0 = mb0;
    Rb = e + d * sin(b0);
}

void ChLineCam::EvaluateCamPoint(double par, ChVector<>& res, double& g, double& q) const {
    double a = par * 2 * CH_C_PI;  // range : par 0..1 -> angle 0...2PI
    double r, f, b, B, fshift, y, ydx, ydxdx, sa, fxalpha, u, uh = 0;
    double sign, signdx, signdxdx;

    // defaults
    g = 0;
    q = 0;
    res = VNULL;

    double radius = internal ? -Rr : +Rr;
    double ecc = negative ? -e : +e;

    fxalpha = a;
    sign = signdx = signdxdx = +1;

    if (negative) {
        sign = signdx = signdxdx = -1;  // reverse sign

        if ((type == CAM_TYPE_ROTATEFOLLOWER) || (type == CAM_TYPE_FLATOSCILLATE)) {
            fxalpha = 2 * CH_C_PI - a;  // also reverse direction
            signdx = signdxdx = +1;
        }
    }

    y = sign * law->Get_y(fxalpha);
    ydx = signdx * law->Get_y_dx(fxalpha);
    ydxdx = signdxdx * law->Get_y_dxdx(fxalpha);

    switch (this->type) {
        case CAM_TYPE_SLIDEFOLLOWER:
            g = atan(ydx / (Rb + y));
            r = sqrt(radius * radius + pow(Rb + y, 2) - 2 * radius * (Rb + y) * cos(g));
            fshift = asin(radius * sin(g) / r);
            if (radius > Rb)
                fshift = CH_C_PI - fshift;
            f = a + fshift;
            q = pow(ydx * ydx + pow(Rb + y, 2), 1.5) / (pow(Rb + y, 2) - ydxdx * (Rb + y) + 2 * (ydx * ydx)) - radius;
            break;
        case CAM_TYPE_ROTATEFOLLOWER:
            b = b0 + y;
            u = atan2((p * sin(b) * (1 - ydx)), (d - p * cos(b) * (1 - ydx)));
            g = CH_C_PI / 2.0 - b - u;
            r = sqrt(pow(p * sin(b) - radius * sin(u), 2) + pow(d - p * cos(b) - radius * cos(u), 2));
            fshift = atan2((p * sin(b) - radius * sin(u)), (d - p * cos(b) - radius * cos(u)));
            f = (a + fshift);
            uh =
                (p * (1 - ydx) * ydx * cos(b + u) - p * ydxdx * sin(b + u)) / (d * cos(u) - p * (1 - ydx) * cos(b + u));
            q = ((p * cos(b0 + y) * (1 - ydx)) + d) / ((1 + uh) * cos(u)) - radius;
            break;
        case CAM_TYPE_ECCENTRICFOLLOWER: {
            double s_dist = Get_s();
            sa = s_dist + y;
            g = atan((ydx - ecc) / (s_dist + y));
            r = sqrt(pow((sa - radius * cos(g)), 2) + pow((ecc + radius * sin(g)), 2));
            fshift = atan((ecc + radius * sin(g)) / (sa - radius * cos(g)));
            if (radius > Rb)
                fshift = CH_C_PI + fshift;
            f = a + fshift;
            q = pow((pow(s_dist + y, 2) + pow(ecc - ydx, 2)), 1.5) /
                    (pow(s_dist + y, 2) + (ecc - ydx) * (ecc - 2 * ydx) - (s_dist + y) * ydxdx) -
                radius;
            break;
        }
        case CAM_TYPE_FLAT:
            g = 0;
            B = ydx;
            r = sqrt(pow(Rb + y, 2) + ydx * ydx);
            f = a + atan2(ydx, (Rb + y));
            q = Rb + y + ydxdx;
            break;
        case CAM_TYPE_FLATOSCILLATE:
            b = b0 + y;
            B = (d * cos(b)) / (1 - ydx);
            g = atan2(ecc, B);
            r = sqrt(pow(d - ecc * sin(b) - B * cos(b), 2) + pow(B * sin(b) - ecc * cos(b), 2));
            f = a + atan2((B * sin(b) - ecc * cos(b)), (d - ecc * sin(b) - B * cos(b)));
            q = (d * sin(b) * (1 - 2 * ydx) + B * ydxdx) / (pow(1 - ydx, 2)) - ecc;
            break;
        default:
            g = r = f = 0;
            break;
    }

    if (negative) {
        if ((type == CAM_TYPE_FLAT) || (type == CAM_TYPE_SLIDEFOLLOWER) || (type == CAM_TYPE_ECCENTRICFOLLOWER))
            f += CH_C_PI;  // polar 180

        if ((type == CAM_TYPE_ROTATEFOLLOWER) || (type == CAM_TYPE_FLATOSCILLATE)) {
            f = -f;  // y mirror
        }
    }

    res.z() = 0;
    res.x() = this->center.x() + r * cos(f + phase);
    res.y() = this->center.y() + r * sin(f + phase);
}

void ChLineCam::Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const {
    double qtmp, gtmp;
    EvaluateCamPoint(parU, pos, gtmp, qtmp);
}

}  // end namespace geometry
}  // end namespace chrono
