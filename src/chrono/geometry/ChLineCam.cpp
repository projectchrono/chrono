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

#include <cmath>

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
    Rb = std::sqrt(p * p + d * d - 2 * p * d * std::cos(b0));
}

void ChLineCam::SetFlatOscillate(double me, double md, double mb0) {
    e = me;
    d = md;
    b0 = mb0;
    Rb = e + d * std::sin(b0);
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
            g = std::atan(ydx / (Rb + y));
            r = std::sqrt(radius * radius + std::pow(Rb + y, 2) - 2 * radius * (Rb + y) * std::cos(g));
            fshift = std::asin(radius * std::sin(g) / r);
            if (radius > Rb)
                fshift = CH_PI - fshift;
            f = a + fshift;
            q = pow(ydx * ydx + std::pow(Rb + y, 2), 1.5) / (std::pow(Rb + y, 2) - ydxdx * (Rb + y) + 2 * (ydx * ydx)) -
                radius;
            break;
        case CamType::ROTATEFOLLOWER:
            b = b0 + y;
            u = std::atan2((p * std::sin(b) * (1 - ydx)), (d - p * std::cos(b) * (1 - ydx)));
            g = CH_PI / 2.0 - b - u;
            r = std::sqrt(std::pow(p * std::sin(b) - radius * std::sin(u), 2) +
                     std::pow(d - p * std::cos(b) - radius * std::cos(u), 2));
            fshift = std::atan2((p * std::sin(b) - radius * std::sin(u)), (d - p * std::cos(b) - radius * std::cos(u)));
            f = (a + fshift);
            uh = (p * (1 - ydx) * ydx * std::cos(b + u) - p * ydxdx * std::sin(b + u)) /
                 (d * std::cos(u) - p * (1 - ydx) * std::cos(b + u));
            q = ((p * std::cos(b0 + y) * (1 - ydx)) + d) / ((1 + uh) * std::cos(u)) - radius;
            break;
        case CamType::ECCENTRICFOLLOWER: {
            double s_dist = std::sqrt(Rb * Rb - e * e);
            sa = s_dist + y;
            g = std::atan((ydx - ecc) / (s_dist + y));
            r = std::sqrt(std::pow((sa - radius * std::cos(g)), 2) + std::pow((ecc + radius * std::sin(g)), 2));
            fshift = std::atan((ecc + radius * std::sin(g)) / (sa - radius * std::cos(g)));
            if (radius > Rb)
                fshift = CH_PI + fshift;
            f = a + fshift;
            q = std::pow((std::pow(s_dist + y, 2) + std::pow(ecc - ydx, 2)), 1.5) /
                    (std::pow(s_dist + y, 2) + (ecc - ydx) * (ecc - 2 * ydx) - (s_dist + y) * ydxdx) -
                radius;
            break;
        }
        case CamType::FLAT:
            g = 0;
            B = ydx;
            r = std::sqrt(std::pow(Rb + y, 2) + ydx * ydx);
            f = a + std::atan2(ydx, (Rb + y));
            q = Rb + y + ydxdx;
            break;
        case CamType::FLATOSCILLATE:
            b = b0 + y;
            B = (d * std::cos(b)) / (1 - ydx);
            g = std::atan2(ecc, B);
            r = std::sqrt(std::pow(d - ecc * std::sin(b) - B * std::cos(b), 2) +
                          std::pow(B * std::sin(b) - ecc * std::cos(b), 2));
            f = a + std::atan2((B * std::sin(b) - ecc * std::cos(b)), (d - ecc * std::sin(b) - B * std::cos(b)));
            q = (d * std::sin(b) * (1 - 2 * ydx) + B * ydxdx) / (std::pow(1 - ydx, 2)) - ecc;
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
    res.x() = this->center.x() + r * std::cos(f + phase);
    res.y() = this->center.y() + r * std::sin(f + phase);

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
