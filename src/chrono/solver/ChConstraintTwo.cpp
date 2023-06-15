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

#include "chrono/solver/ChConstraintTwo.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChConstraintTwo)  // NO! Abstract class

ChConstraintTwo::ChConstraintTwo(const ChConstraintTwo& other) : ChConstraint(other) {
    variables_a = other.variables_a;
    variables_b = other.variables_b;
}

ChConstraintTwo& ChConstraintTwo::operator=(const ChConstraintTwo& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraint::operator=(other);

    this->variables_a = other.variables_a;
    this->variables_b = other.variables_b;

    return *this;
}

void ChConstraintTwo::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConstraintTwo>();

    // serialize the parent class data too
    ChConstraint::ArchiveOut(marchive);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (pointers to variables must be rebound in run-time.)
}

void ChConstraintTwo::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChConstraintTwo>();

    // deserialize the parent class data too
    ChConstraint::ArchiveIn(marchive);

    // stream in all member data:
    // NOTHING INTERESTING TO SERIALIZE (pointers to variables must be rebound in run-time.)
}

}  // end namespace chrono
