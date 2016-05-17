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

#include "chrono/solver/ChConstraintNodeFrictionT.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
ChClassRegister<ChConstraintNodeFrictionT> a_registration_ChConstraintNodeFrictionT;

ChConstraintNodeFrictionT::ChConstraintNodeFrictionT() {
    mode = CONSTRAINT_FRIC;
}

ChConstraintNodeFrictionT::ChConstraintNodeFrictionT(ChVariablesBody* mvariables_a, ChVariablesNode* mvariables_b)
    : ChConstraintTwoGeneric(mvariables_a, mvariables_b) {
    mode = CONSTRAINT_FRIC;
}

ChConstraintNodeFrictionT& ChConstraintNodeFrictionT::operator=(const ChConstraintNodeFrictionT& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintTwoGeneric::operator=(other);

    return *this;
}

double ChConstraintNodeFrictionT::Violation(double mc_i) {
    return 0.0;  //***TO DO*** compute true violation when in sticking?
}

void ChConstraintNodeFrictionT::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);

    // serialize parent class too
    ChConstraintTwoGeneric::StreamOUT(mstream);
}

void ChConstraintNodeFrictionT::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChConstraintTwoGeneric::StreamIN(mstream);
}

}  // end namespace chrono
