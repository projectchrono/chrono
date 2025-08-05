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

#include "chrono/physics/ChContactContainer.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChContactContainer) // NO, abstract class!

ChContactContainer::ChContactContainer(const ChContactContainer& other) : ChPhysicsItem(other) {
    add_contact_callback = other.add_contact_callback;
}

void ChContactContainer::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContactContainer>();
    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);
    // serialize all member data:
}

void ChContactContainer::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContactContainer>();
    // deserialize parent class
    ChPhysicsItem::ArchiveIn(archive_in);
    // stream in all member data:
}

}  // end namespace chrono
