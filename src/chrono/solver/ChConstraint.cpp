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

#include "chrono/solver/ChConstraint.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChConstraint)   // NO! Abstract class

ChConstraint::ChConstraint(const ChConstraint& other) {
    c_i = other.c_i;
    g_i = other.g_i;
    b_i = other.b_i;
    l_i = other.l_i;
    cfm_i = other.cfm_i;
    valid = other.valid;
    disabled = other.disabled;
    redundant = other.redundant;
    broken = other.broken;
    mode = other.mode;
}

ChConstraint& ChConstraint::operator=(const ChConstraint& other) {
    if (&other == this)
        return *this;

    c_i = other.c_i;
    g_i = other.g_i;
    b_i = other.b_i;
    l_i = other.l_i;
    cfm_i = other.cfm_i;
    valid = other.valid;
    active = other.active;
    disabled = other.disabled;
    redundant = other.redundant;
    broken = other.broken;
    mode = other.mode;

    return *this;
}

bool ChConstraint::operator==(const ChConstraint& other) const {
    return other.cfm_i == cfm_i && other.valid == valid && other.active == active && other.disabled == disabled &&
           other.redundant == redundant && other.broken == broken && other.mode == mode;
}

void ChConstraint::Project() {
    if (mode == CONSTRAINT_UNILATERAL) {
        if (l_i < 0.)
            l_i = 0.;
    }
}

double ChConstraint::Violation(double mc_i) {
    if (mode == CONSTRAINT_UNILATERAL) {
        if (mc_i > 0.)
            return 0.;
    }

    return mc_i;
}

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'my_enum_mappers', just to avoid avoiding cluttering of the parent class.
// class my_enum_mappers : public ChConstraint {
//  public:
CH_ENUM_MAPPER_BEGIN(eChConstraintMode);
CH_ENUM_VAL(eChConstraintMode::CONSTRAINT_FREE);
CH_ENUM_VAL(eChConstraintMode::CONSTRAINT_FRIC);
CH_ENUM_VAL(eChConstraintMode::CONSTRAINT_LOCK);
CH_ENUM_VAL(eChConstraintMode::CONSTRAINT_UNILATERAL);
CH_ENUM_MAPPER_END(eChConstraintMode);
//};

void ChConstraint::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChConstraint>();

    // serialize all member data:
    archive_out << CHNVP(cfm_i);
    archive_out << CHNVP(valid);
    archive_out << CHNVP(disabled);
    archive_out << CHNVP(redundant);
    archive_out << CHNVP(broken);
    eChConstraintMode_mapper typemapper;
    archive_out << CHNVP(typemapper(this->mode), "mode");
}

void ChConstraint::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChConstraint>();

    // stream in all member data:
    archive_in >> CHNVP(cfm_i);
    archive_in >> CHNVP(valid);
    archive_in >> CHNVP(disabled);
    archive_in >> CHNVP(redundant);
    archive_in >> CHNVP(broken);
    eChConstraintMode_mapper typemapper;
    archive_in >> CHNVP(typemapper(this->mode), "mode");
    UpdateActiveFlag();

}

}  // end namespace chrono
