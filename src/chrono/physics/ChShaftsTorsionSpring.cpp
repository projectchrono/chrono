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

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsTorsionSpring)

ChShaftsTorsionSpring::ChShaftsTorsionSpring() : stiffness(0), damping(0) {}

ChShaftsTorsionSpring::ChShaftsTorsionSpring(const ChShaftsTorsionSpring& other) : ChShaftsTorque(other) {
    stiffness = other.stiffness;
    damping = other.damping;
}

double ChShaftsTorsionSpring::ComputeTorque() {
    // COMPUTE THE TORQUE HERE!
    return -(GetRelativePos() * stiffness    // the torsional spring term
             + GetRelativePosDt() * damping  // the torsional damper term
    );
}

void ChShaftsTorsionSpring::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsTorsionSpring>();

    // serialize parent class
    ChShaftsTorque::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(stiffness);
    archive_out << CHNVP(damping);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorsionSpring::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsTorsionSpring>();

    // deserialize parent class:
    ChShaftsTorque::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(stiffness);
    archive_in >> CHNVP(damping);
}

}  // end namespace chrono