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
ChClassRegisterABSTRACT<ChLink> a_registration_ChLink;

ChLink::ChLink(const ChLink& other) : ChLinkBase(other) {
    Body1 = NULL;
    Body2 = NULL;

    react_force = other.react_force;
    react_torque = other.react_torque;
}

void ChLink::Copy(ChLink* source) {
    // first copy the parent class data...
    ChLinkBase::Copy(source);

    Body1 = 0;
    Body2 = 0;

    react_force = source->react_force;
    react_torque = source->react_torque;
}

void ChLink::UpdateTime(double time) {
    ChTime = time;
}

void ChLink::Update(double time, bool update_assets) {
    // 1 -
    UpdateTime(time);
}

void ChLink::Update(bool update_assets) {
    Update(ChTime, update_assets);  // use the same time
}

void ChLink::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkBase::ArchiveOUT(marchive);

    // serialize all member data:
}

void ChLink::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLinkBase::ArchiveIN(marchive);

    // deserialize all member data:
}

}  // end namespace chrono
