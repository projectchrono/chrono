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

#include "chrono/geometry/ChLineCam.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineCam)

ChLineCam::ChLineCam() {
    SetComplexity(10);
    this->closed = true;
    type = CamType::SLIDEFOLLOWER;
    Rb = 1.0;
    Rr = 0.0;
    p = 1.8;
    d = 2;
    b0 = CH_PI / 6.0;
    center = VNULL;
    e = 0;
    s = Rb;
    law = chrono_types::make_shared<ChFunctionConst>(0);  // default law = no follower motion
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

void ChLineCam::SetCamRadius(double r) {
    Rb = r;
    if (e > 0.9 * Rb)
        e = 0.9 * Rb;
    if (e < -0.9 * Rb)
        e = -0.9 * Rb;
}

void ChLineCam::SetRotatingFollower(double mp, double md, double mb0) {
    p = mp;
    d = md;
    b0 = mb0;
    Rb = sqrt(p * p + d * d - 2 * p * d * cos(b0));
}

void ChLineCam::SetFlatOscillate(double me, double md, double mb0) {
    e = me;
    d = md;
    b0 = mb0;
    Rb = e + d * sin(b0);
}

ChVector3d ChLineCam::EvaluateCamPoint(double par, double& g, double& q) const {
    double a = par * 2 * CH_PI;  // range : par 0..1 -> angle 0...2PI
    double r, f, b, B, fshift, y, ydx, ydxdx, sa, fxalpha, u, uh = 0;
    double sign, signdx, signdxdx;

    // defaults
    g = 0;
    q = 0;
    ChVector3d res = VNULL;

    double radius = internal ? -Rr : +Rr;
    double ecc = negative ? -e : +e;

    fxalpha = a;
    sign = signdx = signdxdx = +1;

    if (negative) {
        sign = signdx = signdxdx = -1;  // reverse sign

        if ((type == CamType::ROTATEFOLLOWER) || (type == CamType::FLATOSCILLATE)) {
            fxalpha = 2 * CH_PI - a;  // also reverse direction
            signdx = signdxdx = +1;
        }
    }

    y = sign * law->GetVal(fxalpha);
    ydx = signdx * law->GetDer(fxalpha);
    ydxdx = signdxdx * law->GetDer2(fxalpha);

    switch (this->type) {
        case CamType::SLIDEFOLLOWER:
            g = atan(ydx / (Rb + y));
            r = sqrt(radius * radius + pow(Rb + y, 2) - 2 * radius * (Rb + y) * cos(g));
            fshift = asin(radius * sin(g) / r);
            if (radius > Rb)
                fshift = CH_PI - fshift;
            f = a + fshift;
            q = pow(ydx * ydx + pow(Rb + y, 2), 1.5) / (pow(Rb + y, 2) - ydxdx * (Rb + y) + 2 * (ydx * ydx)) - radius;
            break;
        case CamType::ROTATEFOLLOWER:
            b = b0 + y;
            u = atan2((p * sin(b) * (1 - ydx)), (d - p * cos(b) * (1 - ydx)));
            g = CH_PI / 2.0 - b - u;
            r = sqrt(pow(p * sin(b) - radius * sin(u), 2) + pow(d - p * cos(b) - radius * cos(u), 2));
            fshift = atan2((p * sin(b) - radius * sin(u)), (d - p * cos(b) - radius * cos(u)));
            f = (a + fshift);
            uh =
                (p * (1 - ydx) * ydx * cos(b + u) - p * ydxdx * sin(b + u)) / (d * cos(u) - p * (1 - ydx) * cos(b + u));
            q = ((p * cos(b0 + y) * (1 - ydx)) + d) / ((1 + uh) * cos(u)) - radius;
            break;
        case CamType::ECCENTRICFOLLOWER: {
            double s_dist = sqrt(Rb * Rb - e * e);
            sa = s_dist + y;
            g = atan((ydx - ecc) / (s_dist + y));
            r = sqrt(pow((sa - radius * cos(g)), 2) + pow((ecc + radius * sin(g)), 2));
            fshift = atan((ecc + radius * sin(g)) / (sa - radius * cos(g)));
            if (radius > Rb)
                fshift = CH_PI + fshift;
            f = a + fshift;
            q = pow((pow(s_dist + y, 2) + pow(ecc - ydx, 2)), 1.5) /
                    (pow(s_dist + y, 2) + (ecc - ydx) * (ecc - 2 * ydx) - (s_dist + y) * ydxdx) -
                radius;
            break;
        }
        case CamType::FLAT:
            g = 0;
            B = ydx;
            r = sqrt(pow(Rb + y, 2) + ydx * ydx);
            f = a + atan2(ydx, (Rb + y));
            q = Rb + y + ydxdx;
            break;
        case CamType::FLATOSCILLATE:
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
        if ((type == CamType::FLAT) || (type == CamType::SLIDEFOLLOWER) || (type == CamType::ECCENTRICFOLLOWER))
            f += CH_PI;  // polar 180

        if ((type == CamType::ROTATEFOLLOWER) || (type == CamType::FLATOSCILLATE)) {
            f = -f;  // y mirror
        }
    }

    res.z() = 0;
    res.x() = this->center.x() + r * cos(f + phase);
    res.y() = this->center.y() + r * sin(f + phase);

    return res;
}

ChVector3d ChLineCam::Evaluate(double parU) const {
    double qtmp, gtmp;
    return EvaluateCamPoint(parU, gtmp, qtmp);
}

class ChLineenum_mapper : public ChLineCam {
  public:
    CH_ENUM_MAPPER_BEGIN(CamType);
    CH_ENUM_VAL(CamType::SLIDEFOLLOWER);
    CH_ENUM_VAL(CamType::ROTATEFOLLOWER);
    CH_ENUM_VAL(CamType::ECCENTRICFOLLOWER);
    CH_ENUM_VAL(CamType::FLAT);
    CH_ENUM_VAL(CamType::FLATOSCILLATE);
    CH_ENUM_MAPPER_END(CamType);
};

void ChLineCam::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineCam>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:

    ChLineenum_mapper::CamType_mapper mmapper;
    archive_out << CHNVP(mmapper(type), "ChLineCam__Type");
    archive_out << CHNVP(law);
    archive_out << CHNVP(phase);
    archive_out << CHNVP(Rb);
    archive_out << CHNVP(Rr);
    archive_out << CHNVP(p);
    archive_out << CHNVP(d);
    archive_out << CHNVP(b0);
    archive_out << CHNVP(e);
    archive_out << CHNVP(s);
    archive_out << CHNVP(negative);
    archive_out << CHNVP(internal);
    archive_out << CHNVP(center);
}

void ChLineCam::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineCam>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:

    ChLineenum_mapper::CamType_mapper mmapper;
    archive_in >> CHNVP(mmapper(type), "ChLineCam__Type");
    archive_in >> CHNVP(law);
    archive_in >> CHNVP(phase);
    archive_in >> CHNVP(Rb);
    archive_in >> CHNVP(Rr);
    archive_in >> CHNVP(p);
    archive_in >> CHNVP(d);
    archive_in >> CHNVP(b0);
    archive_in >> CHNVP(e);
    archive_in >> CHNVP(s);
    archive_in >> CHNVP(negative);
    archive_in >> CHNVP(internal);
    archive_in >> CHNVP(center);
}

}  // end namespace chrono
