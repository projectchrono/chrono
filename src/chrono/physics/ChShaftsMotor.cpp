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

#include "chrono/physics/ChShaftsMotor.h"

namespace chrono {

void ChShaftsMotor::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsMotor>();

    // serialize parent class
    ChShaftsCouple::ArchiveOut(archive_out);

    // serialize all member data:
}

void ChShaftsMotor::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsMotor>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIn(archive_in);

    // deserialize all member data:
}

}  // end namespace chrono
