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
    // 1 -
    UpdateTime(time);

    // This will update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

void ChLink::Update(bool update_assets) {
    Update(ChTime, update_assets);  // use the same time
}

void ChLink::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLink>();

    // serialize parent class
    ChLinkBase::ArchiveOUT(marchive);

    // serialize all member data:
}

void ChLink::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLink>();

    // deserialize parent class
    ChLinkBase::ArchiveIN(marchive);

    // deserialize all member data:
}

}  // end namespace chrono
