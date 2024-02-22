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

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChLink)   // NO! abstract class!

ChLink::ChLink(const ChLink& other) : ChLinkBase(other) {
    Body1 = NULL;
    Body2 = NULL;

    react_force = other.react_force;
    react_torque = other.react_torque;
}

void ChLink::UpdateTime(double time) {
    ChTime = time;
}

void ChLink::Update(double time, bool update_assets) {
    UpdateTime(time);

    // Update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

void ChLink::Update(bool update_assets) {
    Update(ChTime, update_assets);  // use the same time
}

void ChLink::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLink>();

    // serialize parent class
    ChLinkBase::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(Body1);
    archive_out << CHNVP(Body2);
    //archive_out << CHNVP(react_force);
    //archive_out << CHNVP(react_torque);
}

void ChLink::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChLink>();

    // deserialize parent class
    ChLinkBase::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(Body1);
    archive_in >> CHNVP(Body2);
    //archive_in >> CHNVP(react_force);
    //archive_in >> CHNVP(react_torque);
}

}  // end namespace chrono
