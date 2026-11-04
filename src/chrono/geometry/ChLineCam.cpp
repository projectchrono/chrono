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

#include <cmath>

#include "chrono/geometry/ChLineCam.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineCam)

class ChLineCam_enum_mapper : public ChLineCam {
  public:
    CH_ENUM_MAPPER_BEGIN(TypeFollower);
    CH_ENUM_VAL(TypeFollower::TRANSLATING_INLINE);
    CH_ENUM_VAL(TypeFollower::TRANSLATING_OFFSET);
    CH_ENUM_VAL(TypeFollower::TRANSLATING_FLAT);
    CH_ENUM_VAL(TypeFollower::OSCILLATING_ROLLER);
    CH_ENUM_VAL(TypeFollower::OSCILLATING_FLAT);
    CH_ENUM_MAPPER_END(TypeFollower);
};

ChLineCam::ChLineCam() {
    SetComplexity(10);
    closed = true;
    type = TypeFollower::TRANSLATING_INLINE;
    motlaw = chrono_types::make_shared<ChFunctionConst>(0);  // default motlaw: no follower motion (circular cam)
    phase = 0;
    Rb = 0;
    Rr = 0;
    b = 0;
    d = 0;
    beta0 = 0;
    e = 0;
    negative = false;
    internal = false;
    origin = ChVector2d(0, 0);
}

ChLineCam::ChLineCam(const ChLineCam& source) : ChLine(source) {
    type = source.type;
    motlaw = source.motlaw;
    phase = source.phase;
    Rb = source.Rb;
    Rr = source.Rr;
    b = source.b;
    d = source.d;
    beta0 = source.beta0;
    e = source.e;
    negative = source.negative;
    internal = source.internal;
    origin = source.origin;
}

void ChLineCam::InitializeTranslatingInline(double cam_base_rad, double follower_rad) {
    type = TypeFollower::TRANSLATING_INLINE;
    Rb = cam_base_rad;
    Rr = follower_rad;
}

void ChLineCam::InitializeTranslatingOffset(double cam_base_rad, double follower_rad, double offset) {
    type = TypeFollower::TRANSLATING_OFFSET;
    Rb = cam_base_rad;
    Rr = follower_rad;
    e = offset;
}

void ChLineCam::InitializeTranslatingFlat(double cam_base_rad) {
    type = TypeFollower::TRANSLATING_FLAT;
    Rb = cam_base_rad;
}

void ChLineCam::InitializeOscillatingRoller(double follower_rad, double follower_arm_length, double follower_dist, double follower_angle0) {
    type = TypeFollower::OSCILLATING_ROLLER;
    Rr = follower_rad;
    b = follower_arm_length;
    d = follower_dist;
    beta0 = follower_angle0;
    Rb = std::sqrt(b * b + d * d - 2 * b * d * std::cos(beta0)) - Rr;
}

void ChLineCam::InitializeOscillatingFlat(double follower_arm_offset, double follower_dist, double follower_angle0) {
    type = TypeFollower::OSCILLATING_FLAT;
    e = follower_arm_offset;
    d = follower_dist;
    beta0 = follower_angle0;
    Rb = d * std::sin(beta0) - e;
}

ChVector3d ChLineCam::EvaluateCam(double U, double& theta, double& rho) const {
    const double alpha = U * CH_2PI;  // rescale range from U in [0, 1] to alpha in [0, 2PI]
    const double radius = internal ? -Rr : +Rr;
    const double ecc = negative ? -e : +e;

    // Determine signs and direction
    double sign, signdx, signdxdx;
    sign = signdx = signdxdx = +1;
    double fxalpha = alpha;
    if (negative) {
        sign = signdx = signdxdx = -1;  // reverse sign

        if ((type == TypeFollower::OSCILLATING_ROLLER) || (type == TypeFollower::OSCILLATING_FLAT)) {
            fxalpha = CH_2PI - alpha;  // also reverse direction
            signdx = signdxdx = +1;
        }
    }

    const double y = sign * motlaw->GetVal(fxalpha); // geometric motion motlaw
    const double ydx = signdx * motlaw->GetDer(fxalpha); // geometric derivative
    const double ydxdx = signdxdx * motlaw->GetDer2(fxalpha); // geometric second derivative

    double phi = 0;  // aux parametrization angle [0, 2PI]: if follower radius is nonzero this differs from alpha since contact point is *not* in follower origin
    double r = 0;    // cam profile radius at angle phi
    switch (type) {
        case TypeFollower::TRANSLATING_INLINE: {
            double Rb0_y = (Rb + Rr) + y;
            theta = std::atan2(ydx, Rb0_y);
            r = std::sqrt(radius * radius + Rb0_y * Rb0_y - 2 * radius * Rb0_y * std::cos(theta));
            double alpha_shift = std::asin(radius * std::sin(theta) / r);
            if (radius > Rb) {
                alpha_shift = CH_PI - alpha_shift;
            }
            phi = alpha + alpha_shift;
            rho = std::pow(ydx * ydx + Rb0_y * Rb0_y, 1.5) / (Rb0_y * Rb0_y - Rb0_y * ydxdx + 2 * ydx * ydx) - radius;
            break;
        }
        case TypeFollower::TRANSLATING_OFFSET: {
            double s0 = Rb + Rr;
            double sa = s0 + y;
            theta = std::atan2(ydx - ecc, sa);
            r = std::sqrt(std::pow(sa - radius * std::cos(theta), 2) + std::pow(ecc + radius * std::sin(theta), 2));
            double alpha_shift = std::atan2(ecc + radius * std::sin(theta), sa - radius * std::cos(theta));
            if (radius > Rb) {
                alpha_shift = CH_PI - alpha_shift;
            }
            phi = alpha + alpha_shift;
            double s0_y = s0 + y;
            rho = std::pow((s0_y * s0_y + (ecc - ydx) * (ecc - ydx)), 1.5) / (s0_y * s0_y + (ecc - ydx) * (ecc - 2 * ydx) - s0_y * ydxdx) - radius;
            break;
        }
        case TypeFollower::TRANSLATING_FLAT: {
            theta = 0;
            r = std::sqrt((Rb + y) * (Rb + y) + ydx * ydx);
            phi = alpha + std::atan2(ydx, (Rb + y));
            rho = Rb + y + ydxdx;
            break;
        }
        case TypeFollower::OSCILLATING_ROLLER: {
            double beta = beta0 + y;
            double gamma = std::atan2((b * std::sin(beta) * (1 - ydx)), (d - b * std::cos(beta) * (1 - ydx)));
            theta = CH_PI_2 - beta - gamma;
            r = std::sqrt(std::pow(b * std::sin(beta) - radius * std::sin(gamma), 2) + std::pow(d - b * std::cos(beta) - radius * std::cos(gamma), 2));
            double alpha_shift = std::atan2((b * std::sin(beta) - radius * std::sin(gamma)), (d - b * std::cos(beta) - radius * std::cos(gamma)));
            phi = alpha + alpha_shift;
            double gamma_der = (b * (1 - ydx) * ydx * std::cos(beta + gamma) - b * ydxdx * std::sin(beta + gamma)) / (d * std::cos(gamma) - b * (1 - ydx) * std::cos(beta + gamma));
            rho = ((b * std::cos(beta0 + y) * (1 - ydx)) + d) / ((1 + gamma_der) * std::cos(gamma)) - radius;
            break;
        }
        case TypeFollower::OSCILLATING_FLAT: {
            double beta = beta0 + y;
            double bb = (d * std::cos(beta)) / (1 - ydx); // follower plate should have length b_min < bb < b_max
            theta = std::atan2(ecc, bb);
            r = std::sqrt(std::pow(d - ecc * std::sin(beta) - bb * std::cos(beta), 2) + std::pow(bb * std::sin(beta) - ecc * std::cos(beta), 2));
            double alpha_shift = std::atan2(bb * std::sin(beta) - ecc * std::cos(beta), d - ecc * std::sin(beta) - bb * std::cos(beta));
            phi = alpha + alpha_shift;
            rho = (d * std::sin(beta) * (1 - 2 * ydx) + bb * ydxdx) / ((1 - ydx) * (1 - ydx)) - ecc;
            break;
        }
        default: {
            theta = 0;
            rho = 0;
            break;
        }
    }

    if (negative) {
        if ((type == TypeFollower::TRANSLATING_FLAT) || (type == TypeFollower::TRANSLATING_INLINE) || (type == TypeFollower::TRANSLATING_OFFSET))
            phi += CH_PI;  // polar 180

        if ((type == TypeFollower::OSCILLATING_ROLLER) || (type == TypeFollower::OSCILLATING_FLAT)) {
            phi = -phi;  // y mirror
        }
    }

    return ChVector3d(origin.x() + r * std::cos(phi + phase), origin.y() + r * std::sin(phi + phase), 0.0);
}

ChVector3d ChLineCam::Evaluate(double parU) const {
    double qtmp, gtmp;
    return EvaluateCam(parU, gtmp, qtmp);
}

void ChLineCam::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineCam>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:

    ChLineCam_enum_mapper::TypeFollower_mapper mapper;
    archive_out << CHNVP(mapper(type), "ChLineCam__Type");
    archive_out << CHNVP(origin);
    archive_out << CHNVP(motlaw);
    archive_out << CHNVP(phase);
    archive_out << CHNVP(Rb);
    archive_out << CHNVP(negative);
    archive_out << CHNVP(internal);
    archive_out << CHNVP(Rr);
    archive_out << CHNVP(e);
    archive_out << CHNVP(b);
    archive_out << CHNVP(d);
    archive_out << CHNVP(beta0);
}

void ChLineCam::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineCam>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:

    ChLineCam_enum_mapper::TypeFollower_mapper mapper;
    archive_in >> CHNVP(mapper(type), "ChLineCam__Type");
    archive_in >> CHNVP(origin);
    archive_in >> CHNVP(motlaw);
    archive_in >> CHNVP(phase);
    archive_in >> CHNVP(Rb);
    archive_in >> CHNVP(negative);
    archive_in >> CHNVP(internal);
    archive_in >> CHNVP(Rr);
    archive_in >> CHNVP(e);
    archive_in >> CHNVP(b);
    archive_in >> CHNVP(d);
    archive_in >> CHNVP(beta0);
}

}  // end namespace chrono
