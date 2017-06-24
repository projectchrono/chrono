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

#include "chrono/solver/ChConstraintThree.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChConstraintThree)  // NO! Abstract class

ChConstraintThree::ChConstraintThree(const ChConstraintThree& other) : ChConstraint(other) {
    variables_a = other.variables_a;
    variables_b = other.variables_b;
    variables_c = other.variables_c;
}

ChConstraintThree& ChConstraintThree::operator=(const ChConstraintThree& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraint::operator=(other);

    this->variables_a = other.variables_a;
    this->variables_b = other.variables_b;
    this->variables_c = other.variables_c;

    return *this;
}

void ChConstraintThree::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);

    // serialize parent class too
    ChConstraint::StreamOUT(mstream);

    // stream out all member data
    // NOTHING INTERESTING TO SERIALIZE (pointers to variables must be rebound in run-time.)
}

void ChConstraintThree::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChConstraint::StreamIN(mstream);

    // stream in all member data
    // NOTHING INTERESTING TO DESERIALIZE (pointers to variables must be rebound in run-time.)
}

}  // end namespace chrono
