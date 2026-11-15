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

// =============================================================================
// ChLineCam
// =============================================================================
ChLineCam::ChLineCam() {
    SetComplexity(10);
    SetClosed(true);
    motlaw = chrono_types::make_shared<ChFunctionConst>(0);  // default: no follower motion (circular cam)
}

ChLineCam::ChLineCam(const ChLineCam& source) : ChLine(source) {
    type = source.type;
    motlaw = source.motlaw;
    phase = source.phase;
    Rb = source.Rb;
    negative = source.negative;
    internal = source.internal;
    origin = source.origin;
}

ChVector3d ChLineCam::Evaluate(double parU) const {
    double tmp1, tmp2;
    return EvaluateCam(parU, tmp1, tmp2);
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
}

void ChLineCam::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChLineCam>();
    ChLine::ArchiveIn(archive_in);

    ChLineCam_enum_mapper::TypeFollower_mapper mapper;
    archive_in >> CHNVP(mapper(type), "ChLineCam__Type");
    archive_in >> CHNVP(origin);
    archive_in >> CHNVP(motlaw);
    archive_in >> CHNVP(phase);
    archive_in >> CHNVP(Rb);
    archive_in >> CHNVP(negative);
    archive_in >> CHNVP(internal);
}

ChVector3d ChLineCam::CalcProfilePoint(double r, double phi) const {
    double phi_shift = phi + phase;
    return ChVector3d(origin.x() + r * std::cos(phi_shift), origin.y() + r * std::sin(phi_shift), 0.0);
}

// =============================================================================
// ChLineCamTranslatingInline
// =============================================================================
ChLineCamTranslatingInline::ChLineCamTranslatingInline(double cam_base_rad, double follower_rad) {
    Initialize(cam_base_rad, follower_rad);
}

void ChLineCamTranslatingInline::Initialize(double cam_base_rad, double follower_rad) {
    type = TypeFollower::TRANSLATING_INLINE;
    Rb = cam_base_rad;
    Rr = follower_rad;
}

ChVector3d ChLineCamTranslatingInline::EvaluateCam(double U, double& theta, double& rho) const {
    double alpha = U * CH_2PI;               // rescale range from U in [0, 1] to alpha in [0, 2PI]
    double y = motlaw->GetVal(alpha);        // geometric motlaw
    double y_der = motlaw->GetDer(alpha);    // geometric derivative
    double y_der2 = motlaw->GetDer2(alpha);  // geometric second derivative
    if (negative) {
        y *= -1;
        y_der *= -1;
        y_der2 *= -1;
    }

    double Rr_s = internal ? -Rr : +Rr;
    double Rb0_y = (Rb + Rr_s) + y;
    theta = std::atan2(y_der, Rb0_y);
    double r = std::sqrt(Rr_s * Rr_s + Rb0_y * Rb0_y - 2 * Rr_s * Rb0_y * std::cos(theta));  // cam radius at angle phi
    double alpha_shift = std::asin(Rr_s * std::sin(theta) / r);
    if (Rr_s > Rb) {
        alpha_shift = CH_PI - alpha_shift;
    }
    double phi = alpha + alpha_shift;  // aux parametrization angle in [0, 2PI]
    rho = std::pow(y_der * y_der + Rb0_y * Rb0_y, 1.5) / (Rb0_y * Rb0_y - Rb0_y * y_der2 + 2 * y_der * y_der) - Rr_s;

    if (negative) {
        phi += CH_PI;  // polar 180
    }

    return CalcProfilePoint(r, phi);
}

void ChLineCamTranslatingInline::ArchiveOut(ChArchiveOut& archive_out) {
    ChLineCam::ArchiveOut(archive_out);
    archive_out << CHNVP(Rr);
}

void ChLineCamTranslatingInline::ArchiveIn(ChArchiveIn& archive_in) {
    ChLineCam::ArchiveIn(archive_in);
    archive_in >> CHNVP(Rr);
}

// =============================================================================
// ChLineCamTranslatingOffset
// =============================================================================
ChLineCamTranslatingOffset::ChLineCamTranslatingOffset(double cam_base_rad, double follower_rad, double offset) {
    Initialize(cam_base_rad, follower_rad, offset);
}

void ChLineCamTranslatingOffset::Initialize(double cam_base_rad, double follower_rad, double offset) {
    type = TypeFollower::TRANSLATING_OFFSET;
    Rb = cam_base_rad;
    Rr = follower_rad;
    e = offset;
}

ChVector3d ChLineCamTranslatingOffset::EvaluateCam(double U, double& theta, double& rho) const {
    double alpha = U * CH_2PI;               // rescale range from U in [0, 1] to alpha in [0, 2PI]
    double y = motlaw->GetVal(alpha);        // geometric motlaw
    double y_der = motlaw->GetDer(alpha);    // geometric derivative
    double y_der2 = motlaw->GetDer2(alpha);  // geometric second derivative
    if (negative) {
        y *= -1;
        y_der *= -1;
        y_der2 *= -1;
    }

    double Rr_s = internal ? -Rr : +Rr;
    double e_s = negative ? -e : +e;
    double s0 = Rb + Rr_s;
    double s = s0 + y;
    theta = std::atan2(y_der - e_s, s);
    double r = std::sqrt(std::pow(s - Rr_s * std::cos(theta), 2) + std::pow(e_s + Rr_s * std::sin(theta), 2));  // cam radius at angle phi
    double alpha_shift = std::atan2(e_s + Rr_s * std::sin(theta), s - Rr_s * std::cos(theta));
    if (Rr_s > Rb) {
        alpha_shift = CH_PI - alpha_shift;
    }
    double phi = alpha + alpha_shift;  // aux parametrization angle in [0, 2PI]
    double s0_y = s0 + y;
    rho = std::pow((s0_y * s0_y + (e_s - y_der) * (e_s - y_der)), 1.5) / (s0_y * s0_y + (e_s - y_der) * (e_s - 2 * y_der) - s0_y * y_der2) - Rr_s;

    if (negative) {
        phi += CH_PI;  // polar 180
    }

    return CalcProfilePoint(r, phi);
}

void ChLineCamTranslatingOffset::ArchiveOut(ChArchiveOut& archive_out) {
    ChLineCam::ArchiveOut(archive_out);
    archive_out << CHNVP(Rr);
    archive_out << CHNVP(e);
}

void ChLineCamTranslatingOffset::ArchiveIn(ChArchiveIn& archive_in) {
    ChLineCam::ArchiveIn(archive_in);
    archive_in >> CHNVP(Rr);
    archive_in >> CHNVP(e);
}

// =============================================================================
// ChLineCamTranslatingFlat
// =============================================================================
ChLineCamTranslatingFlat::ChLineCamTranslatingFlat(double cam_base_rad) {
    Initialize(cam_base_rad);
}

void ChLineCamTranslatingFlat::Initialize(double cam_base_rad) {
    type = TypeFollower::TRANSLATING_FLAT;
    Rb = cam_base_rad;
}

ChVector3d ChLineCamTranslatingFlat::EvaluateCam(double U, double& theta, double& rho) const {
    double alpha = U * CH_2PI;               // rescale range from U in [0, 1] to alpha in [0, 2PI]
    double y = motlaw->GetVal(alpha);        // geometric motlaw
    double y_der = motlaw->GetDer(alpha);    // geometric derivative
    double y_der2 = motlaw->GetDer2(alpha);  // geometric second derivative
    if (negative) {
        y *= -1;
        y_der *= -1;
        y_der2 *= -1;
    }

    theta = 0;
    double r = std::sqrt((Rb + y) * (Rb + y) + y_der * y_der);  // cam radius at angle phi
    double phi = alpha + std::atan2(y_der, (Rb + y));           // aux parametrization angle in [0, 2PI]
    rho = Rb + y + y_der2;

    if (negative) {
        phi += CH_PI;  // polar 180
    }

    return CalcProfilePoint(r, phi);
}

void ChLineCamTranslatingFlat::ArchiveOut(ChArchiveOut& archive_out) {
    ChLineCam::ArchiveOut(archive_out);
}

void ChLineCamTranslatingFlat::ArchiveIn(ChArchiveIn& archive_in) {
    ChLineCam::ArchiveIn(archive_in);
}

// =============================================================================
// ChLineCamOscillatingRoller
// =============================================================================
ChLineCamOscillatingRoller::ChLineCamOscillatingRoller(double follower_rad, double follower_arm_length, double follower_dist, double follower_angle0) {
    Initialize(follower_rad, follower_arm_length, follower_dist, follower_angle0);
}

void ChLineCamOscillatingRoller::Initialize(double follower_rad, double follower_arm_length, double follower_dist, double follower_angle0) {
    type = TypeFollower::OSCILLATING_ROLLER;
    Rr = follower_rad;
    b = follower_arm_length;
    d = follower_dist;
    beta0 = follower_angle0;
    Rb = std::sqrt(b * b + d * d - 2 * b * d * std::cos(beta0)) - Rr;
}

ChVector3d ChLineCamOscillatingRoller::EvaluateCam(double U, double& theta, double& rho) const {
    double alpha = U * CH_2PI;  // rescale range from U in [0, 1] to alpha in [0, 2PI]
    double alpha_s = alpha;
    if (negative) {
        alpha_s = CH_2PI - alpha;  // reverse direction
    }
    double y = motlaw->GetVal(alpha_s);        // geometric motlaw
    double y_der = motlaw->GetDer(alpha_s);    // geometric derivative
    double y_der2 = motlaw->GetDer2(alpha_s);  // geometric second derivative
    if (negative) {
        y *= -1;
    }

    double Rr_s = internal ? -Rr : +Rr;
    double beta = beta0 + y;
    double gamma = std::atan2((b * std::sin(beta) * (1 - y_der)), (d - b * std::cos(beta) * (1 - y_der)));
    theta = CH_PI_2 - beta - gamma;
    double r = std::sqrt(std::pow(b * std::sin(beta) - Rr_s * std::sin(gamma), 2) + std::pow(d - b * std::cos(beta) - Rr_s * std::cos(gamma), 2));  // cam radius at angle phi
    double alpha_shift = std::atan2((b * std::sin(beta) - Rr_s * std::sin(gamma)), (d - b * std::cos(beta) - Rr_s * std::cos(gamma)));
    double phi = alpha + alpha_shift;  // aux parametrization angle in [0, 2PI]
    double gamma_der = (b * (1 - y_der) * y_der * std::cos(beta + gamma) - b * y_der2 * std::sin(beta + gamma)) / (d * std::cos(gamma) - b * (1 - y_der) * std::cos(beta + gamma));
    rho = ((b * std::cos(beta0 + y) * (1 - y_der)) + d) / ((1 + gamma_der) * std::cos(gamma)) - Rr_s;

    if (negative) {
        phi = -phi;  // y mirror
    }

    return CalcProfilePoint(r, phi);
}

void ChLineCamOscillatingRoller::ArchiveOut(ChArchiveOut& archive_out) {
    ChLineCam::ArchiveOut(archive_out);
    archive_out << CHNVP(Rr);
    archive_out << CHNVP(b);
    archive_out << CHNVP(d);
    archive_out << CHNVP(beta0);
}

void ChLineCamOscillatingRoller::ArchiveIn(ChArchiveIn& archive_in) {
    ChLineCam::ArchiveIn(archive_in);
    archive_in >> CHNVP(Rr);
    archive_in >> CHNVP(b);
    archive_in >> CHNVP(d);
    archive_in >> CHNVP(beta0);
}

// =============================================================================
// ChLineCamOscillatingFlat
// =============================================================================
void ChLineCamOscillatingFlat::Initialize(double follower_arm_offset, double follower_dist, double follower_angle0) {
    type = TypeFollower::OSCILLATING_FLAT;
    e = follower_arm_offset;
    d = follower_dist;
    beta0 = follower_angle0;
    Rb = d * std::sin(beta0) - e;
}

ChVector3d ChLineCamOscillatingFlat::EvaluateCam(double U, double& theta, double& rho) const {
    double alpha = U * CH_2PI;  // rescale range from U in [0, 1] to alpha in [0, 2PI]
    double alpha_s = alpha;
    if (negative) {
        alpha_s = CH_2PI - alpha;  // reverse direction
    }
    double y = motlaw->GetVal(alpha_s);        // geometric motlaw
    double y_der = motlaw->GetDer(alpha_s);    // geometric derivative
    double y_der2 = motlaw->GetDer2(alpha_s);  // geometric second derivative
    if (negative) {
        y *= -1;
    }

    double e_s = negative ? -e : +e;
    double beta = beta0 + y;
    double bb = (d * std::cos(beta)) / (1 - y_der);  // follower plate should have length b_min < bb < b_max
    theta = std::atan2(e_s, bb);
    double r = std::sqrt(std::pow(d - e_s * std::sin(beta) - bb * std::cos(beta), 2) + std::pow(bb * std::sin(beta) - e_s * std::cos(beta), 2));  // cam radius at angle phi
    double alpha_shift = std::atan2(bb * std::sin(beta) - e_s * std::cos(beta), d - e_s * std::sin(beta) - bb * std::cos(beta));
    double phi = alpha + alpha_shift;  // aux parametrization angle in [0, 2PI]
    rho = (d * std::sin(beta) * (1 - 2 * y_der) + bb * y_der2) / ((1 - y_der) * (1 - y_der)) - e_s;

    if (negative) {
        phi = -phi;  // y mirror
    }

    return CalcProfilePoint(r, phi);
}

void ChLineCamOscillatingFlat::ArchiveOut(ChArchiveOut& archive_out) {
    ChLineCam::ArchiveOut(archive_out);
    archive_out << CHNVP(e);
    archive_out << CHNVP(d);
    archive_out << CHNVP(beta0);
}

void ChLineCamOscillatingFlat::ArchiveIn(ChArchiveIn& archive_in) {
    ChLineCam::ArchiveIn(archive_in);
    archive_in >> CHNVP(e);
    archive_in >> CHNVP(d);
    archive_in >> CHNVP(beta0);
}

}  // end namespace chrono
