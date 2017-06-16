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

#include "chrono/physics/ChNodeXYZ.h"

namespace chrono {

ChNodeXYZ::ChNodeXYZ() : pos(VNULL), pos_dt(VNULL), pos_dtdt(VNULL) {}

ChNodeXYZ::ChNodeXYZ(const ChVector<>& initial_pos) : pos(initial_pos), pos_dt(VNULL), pos_dtdt(VNULL) {}

ChNodeXYZ::ChNodeXYZ(const ChNodeXYZ& other) : ChNodeBase(other) {
    pos = other.pos;
    pos_dt = other.pos_dt;
    pos_dtdt = other.pos_dtdt;
}

ChNodeXYZ& ChNodeXYZ::operator=(const ChNodeXYZ& other) {
    if (&other == this)
        return *this;

    ChNodeBase::operator=(other);

    pos = other.pos;
    pos_dt = other.pos_dt;
    pos_dtdt = other.pos_dtdt;

    return *this;
}

void ChNodeXYZ::ComputeNF(
    const double U,              // x coordinate of application point in absolute space
    const double V,              // y coordinate of application point in absolute space
    const double W,              // z coordinate of application point in absolute space
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is 3, it is Force x,y,z in absolute coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    // ChVector<> abs_pos(U,V,W); not needed, nodes has no torque. Assuming load is applied to node center
    ChVector<> absF = F.ClipVector(0, 0);
    Qi.PasteVector(absF, 0, 0);
    detJ = 1;  // not needed because not used in quadrature.
}

void ChNodeXYZ::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeXYZ>();

    // serialize parent class
    ChNodeBase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(pos);
    marchive << CHNVP(pos_dt);
    marchive << CHNVP(pos_dtdt);
}

/// Method to allow de serialization of transient data from archives.
void ChNodeXYZ::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChNodeXYZ>();

    // deserialize parent class:
    ChNodeBase::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(pos);
    marchive >> CHNVP(pos_dt);
    marchive >> CHNVP(pos_dtdt);
}

}  // end namespace chrono
