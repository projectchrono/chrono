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
#include "chrono/physics/ChLinkBase.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChLinkBase)   // NO! Abstract class!

ChLinkBase::ChLinkBase(const ChLinkBase& other) : ChPhysicsItem(other) {
    disabled = other.disabled;
    valid = other.valid;
    broken = other.broken;
}

void ChLinkBase::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkBase>();

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(disabled);
    marchive << CHNVP(valid);
    marchive << CHNVP(broken);
}

void ChLinkBase::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkBase>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(disabled);
    marchive >> CHNVP(valid);
    marchive >> CHNVP(broken);
}

}  // end namespace chrono
