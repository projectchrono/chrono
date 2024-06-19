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
#include "chrono/physics/ChShaftsAppliedTorque.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsAppliedTorque)

ChShaftsAppliedTorque::ChShaftsAppliedTorque(const ChShaftsAppliedTorque& other) : ChShaftsTorque(other) {}

double ChShaftsAppliedTorque::ComputeTorque() {
    // Simply return the user-specified torque
    return torque;
}

void ChShaftsAppliedTorque::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsAppliedTorque>();

    // serialize parent class
    ChShaftsTorque::ArchiveOut(archive_out);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsAppliedTorque::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsAppliedTorque>();

    // deserialize parent class:
    ChShaftsTorque::ArchiveIn(archive_in);

    // deserialize all member data:
}

}  // end namespace chrono
