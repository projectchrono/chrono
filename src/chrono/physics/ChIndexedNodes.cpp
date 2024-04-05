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

#include <cstdlib>
#include <algorithm>

#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChIndexedNodes) // NO! abstract class!

void ChIndexedNodes::ArchiveOut(ChArchiveOut& archive_out) {
    // class version number
    archive_out.VersionWrite<ChIndexedNodes>();

    // serialize parent class too
    ChPhysicsItem::ArchiveOut(archive_out);

    // stream out all member data
}

void ChIndexedNodes::ArchiveIn(ChArchiveIn& archive_in) {
    // class version number
    /*int version =*/archive_in.VersionRead<ChIndexedNodes>();

    // deserialize parent class too
    ChPhysicsItem::ArchiveIn(archive_in);

    // stream in all member data
}

}  // end namespace chrono
