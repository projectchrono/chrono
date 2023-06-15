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

#include "chrono/physics/ChProximityContainer.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChProximityContainer)  // NO! Abstract class

ChProximityContainer::ChProximityContainer(const ChProximityContainer& other) : ChPhysicsItem(other) {
    add_proximity_callback = other.add_proximity_callback;
    report_proximity_callback = other.report_proximity_callback;
}

void ChProximityContainer::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite(1);
    // serialize parent class
    ChPhysicsItem::ArchiveOut(marchive);
    // serialize all member data:
}

void ChProximityContainer::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead();
    // deserialize parent class
    ChPhysicsItem::ArchiveIn(marchive);
    // stream in all member data:
}

}  // end namespace chrono
