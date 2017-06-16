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

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsTorsionSpring)

ChShaftsTorsionSpring::ChShaftsTorsionSpring() : stiffness(0), damping(0) {}

ChShaftsTorsionSpring::ChShaftsTorsionSpring(const ChShaftsTorsionSpring& other) : ChShaftsTorqueBase(other) {
    stiffness = other.stiffness;
    damping = other.damping;
}

double ChShaftsTorsionSpring::ComputeTorque() {
    // COMPUTE THE TORQUE HERE!
    return -(GetRelativeRotation() * stiffness     // the torsional spring term
             + GetRelativeRotation_dt() * damping  // the torsional damper term
             );
}

// FILE I/O

void ChShaftsTorsionSpring::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsTorsionSpring>();

    // serialize parent class
    ChShaftsTorqueBase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(stiffness);
    marchive << CHNVP(damping);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorsionSpring::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsTorsionSpring>();

    // deserialize parent class:
    ChShaftsTorqueBase::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(stiffness);
    marchive >> CHNVP(damping);
}

}  // end namespace chrono