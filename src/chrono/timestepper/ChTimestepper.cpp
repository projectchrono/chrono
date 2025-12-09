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

#include "chrono/timestepper/ChTimestepper.h"

namespace chrono {

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'ChTimestepper_Type_enum_mapper', just to avoid avoiding cluttering of the parent class.
class ChTimestepper_Type_enum_mapper : public ChTimestepper {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::EULER_EXPLICIT_I);
    CH_ENUM_VAL(Type::EULER_EXPLICIT_II);
    CH_ENUM_VAL(Type::EULER_SEMI_IMPLICIT);
    CH_ENUM_VAL(Type::RUNGE_KUTTA);
    CH_ENUM_VAL(Type::HEUN);
    CH_ENUM_VAL(Type::LEAPFROG);
    CH_ENUM_VAL(Type::EULER_IMPLICIT);
    CH_ENUM_VAL(Type::EULER_IMPLICIT_LINEARIZED);
    CH_ENUM_VAL(Type::EULER_IMPLICIT_PROJECTED);
    CH_ENUM_VAL(Type::TRAPEZOIDAL);
    CH_ENUM_VAL(Type::TRAPEZOIDAL_LINEARIZED);
    CH_ENUM_VAL(Type::NEWMARK);
    CH_ENUM_VAL(Type::HHT);
    CH_ENUM_VAL(Type::CUSTOM);
    CH_ENUM_MAPPER_END(Type);
};

// -----------------------------------------------------------------------------

ChTimestepper::ChTimestepper(ChIntegrable* intgr) : T(0), verbose(false), Qc_do_clamp(false), Qc_clamping(0) {}

void ChTimestepper::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChTimestepper>();
    // method type:
    ChTimestepper_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    archive << CHNVP(typemapper(type), "timestepper_type");
    // serialize all member data:
    archive << CHNVP(verbose);
}

void ChTimestepper::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChTimestepper>();
    // method type:
    ChTimestepper_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    archive >> CHNVP(typemapper(type), "timestepper_type");
    // stream in all member data:
    archive >> CHNVP(verbose);
}

std::string ChTimestepper::GetTypeAsString(Type type) {
    switch (type) {
        case Type::EULER_EXPLICIT_I:
            return "EULER_EXPLICIT_I";
        case Type::EULER_EXPLICIT_II:
            return "EULER_EXPLICIT_II";
        case Type::EULER_SEMI_IMPLICIT:
            return "EULER_SEMI_IMPLICIT";
        case Type::RUNGE_KUTTA:
            return "RUNGE_KUTTA";
        case Type::HEUN:
            return "HEUN";
        case Type::LEAPFROG:
            return "LEAPFROG";

        case Type::EULER_IMPLICIT:
            return "EULER_IMPLICIT";
        case Type::EULER_IMPLICIT_LINEARIZED:
            return "EULER_IMPLICIT_LINEARIZED";
        case Type::EULER_IMPLICIT_PROJECTED:
            return "EULER_IMPLICIT_PROJECTED";
        case Type::TRAPEZOIDAL:
            return "TRAPEZOIDAL";
        case Type::TRAPEZOIDAL_LINEARIZED:
            return "TRAPEZOIDAL_LINEARIZED";
        case Type::NEWMARK:
            return "NEWMARK";
        case Type::HHT:
            return "HHT";

        case Type::CUSTOM:
            return "CUSTOM";
    }

    return "UNKNOWN";
}

// -----------------------------------------------------------------------------

ChTimestepperIorder::ChTimestepperIorder(ChIntegrable* intgr) : integrable(intgr) {
    Y.setZero(1, intgr);
    dYdt.setZero(1, intgr);
}

void ChTimestepperIorder::SetIntegrable(ChIntegrable* intgr) {
    integrable = intgr;
    Y.setZero(1, intgr);
    dYdt.setZero(1, intgr);
}

// -----------------------------------------------------------------------------

ChTimestepperIIorder::ChTimestepperIIorder(ChIntegrableIIorder* intgr) : integrable(intgr) {
    X.setZero(1, intgr);
    V.setZero(1, intgr);
    A.setZero(1, intgr);
}

void ChTimestepperIIorder::SetIntegrable(ChIntegrableIIorder* intgr) {
    integrable = intgr;
    X.setZero(1, intgr);
    V.setZero(1, intgr);
    A.setZero(1, intgr);
}

}  // end namespace chrono
