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

#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChProbe.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChProbe)

void ChProbe::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChProbe>();

    // serialize parent class
    ChObj::ArchiveOUT(marchive);
}

/// Method to allow de serialization of transient data from archives.
void ChProbe::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChProbe>();

    // deserialize parent class
    ChObj::ArchiveIN(marchive);
}

}  // end namespace chrono
