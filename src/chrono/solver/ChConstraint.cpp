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

ChConstraint::ChConstraint()
    : c_i(0),
      l_i(0),
      b_i(0),
      cfm_i(0),
      valid(false),
      disabled(false),
      redundant(false),
      broken(false),
      active(true),
      mode(Mode::LOCK),
      g_i(0) {}

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
    if (mode == Mode::UNILATERAL) {
        if (l_i < 0.)
            l_i = 0.;
    }
}

double ChConstraint::Violation(double mc_i) {
    if (mode == Mode::UNILATERAL) {
        if (mc_i > 0.)
            return 0.;
    }

    return mc_i;
}

void ChConstraint::UpdateActiveFlag() {
    this->active = (valid && !disabled && !redundant && !broken && mode != (Mode::FREE));
}

// To avoid putting the following mapper macro inside the class definition,
// enclose macros in local 'ChConstraint_Mode_enum_mapper'.
class ChConstraint_Mode_enum_mapper : public ChConstraint {
  public:
    CH_ENUM_MAPPER_BEGIN(Mode);
    CH_ENUM_VAL(Mode::FREE);
    CH_ENUM_VAL(Mode::LOCK);
    CH_ENUM_VAL(Mode::UNILATERAL);
    CH_ENUM_VAL(Mode::FRICTION);
    CH_ENUM_MAPPER_END(Mode);
};

void ChConstraint::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChConstraint>();

    // serialize all member data:
    archive_out << CHNVP(cfm_i);
    archive_out << CHNVP(valid);
    archive_out << CHNVP(disabled);
    archive_out << CHNVP(redundant);
    archive_out << CHNVP(broken);
    ChConstraint_Mode_enum_mapper::Mode_mapper modemapper;
    archive_out << CHNVP(modemapper(mode), "mode");
}

void ChConstraint::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChConstraint>();

    // stream in all member data:
    archive_in >> CHNVP(cfm_i);
    archive_in >> CHNVP(valid);
    archive_in >> CHNVP(disabled);
    archive_in >> CHNVP(redundant);
    archive_in >> CHNVP(broken);
    ChConstraint_Mode_enum_mapper::Mode_mapper modemapper;
    archive_in >> CHNVP(modemapper(mode), "mode");


    UpdateActiveFlag();
}

}  // end namespace chrono
