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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono_fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

ChNodeFEAxyz::ChNodeFEAxyz(ChVector<> initial_pos) : ChNodeXYZ(initial_pos), X0(initial_pos), Force(VNULL) {
    variables.SetNodeMass(0);
}

ChNodeFEAxyz::ChNodeFEAxyz(const ChNodeFEAxyz& other) : ChNodeFEAbase(other), ChNodeXYZ(other) {
    X0 = other.X0;
    Force = other.Force;
    variables = other.variables;
}

ChNodeFEAxyz& ChNodeFEAxyz::operator=(const ChNodeFEAxyz& other) {
    if (&other == this)
        return *this;

    ChNodeFEAbase::operator=(other);
    ChNodeFEAxyz::operator=(other);

    X0 = other.X0;
    Force = other.Force;
    variables = other.variables;

    return *this;
}

void ChNodeFEAxyz::Relax() {
    X0 = pos;
    SetNoSpeedNoAcceleration();
}

void ChNodeFEAxyz::SetNoSpeedNoAcceleration() {
    pos_dt = VNULL;
    pos_dtdt = VNULL;
}

void ChNodeFEAxyz::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAxyz>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOUT(marchive);
    // serialize parent class
    ChNodeXYZ::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(X0);
    marchive << CHNVP(Force);
}

void ChNodeFEAxyz::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChNodeFEAxyz>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIN(marchive);
    // serialize parent class
    ChNodeXYZ::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(X0);
    marchive >> CHNVP(Force);
}

}  // end namespace fea
}  // end namespace chrono
