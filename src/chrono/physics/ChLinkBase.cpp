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

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChLinkBase.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChLinkBase)   // NO! Abstract class!

ChLinkBase::ChLinkBase(const ChLinkBase& other) : ChPhysicsItem(other) {
    disabled = other.disabled;
    valid = other.valid;
    broken = other.broken;
}

void ChLinkBase::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkBase>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(disabled);
    archive_out << CHNVP(valid);
    archive_out << CHNVP(broken);
}

void ChLinkBase::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkBase>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(disabled);
    archive_in >> CHNVP(valid);
    archive_in >> CHNVP(broken);
}

}  // end namespace chrono
