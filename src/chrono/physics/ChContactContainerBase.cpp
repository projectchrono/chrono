//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "physics/ChContactContainerBase.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChContactContainerBase> a_registration_ChContactContainerBase;

ChVector<> ChContactContainerBase::GetContactableForce(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.force;
    }
    return ChVector<>(0);
}

ChVector<> ChContactContainerBase::GetContactableTorque(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.torque;
    }
    return ChVector<>(0);
}

void ChContactContainerBase::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite(1);
    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);
    // serialize all member data:
}

void ChContactContainerBase::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead();
    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);
    // stream in all member data:
}

}  // end namespace chrono
