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

#include "physics/ChLink.h"
#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"
#include "physics/ChExternalObject.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChLink> a_registration_ChLink;

// BUILDERS
ChLink::ChLink() {
    Body1 = NULL;
    Body2 = NULL;

    react_force = VNULL;
    react_torque = VNULL;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

// DESTROYER
ChLink::~ChLink() {
}

void ChLink::Copy(ChLink* source) {
    // first copy the parent class data...
    ChLinkBase::Copy(source);

    Body1 = 0;
    Body2 = 0;
    system = 0;

    react_force = source->react_force;
    react_torque = source->react_torque;
}

////////////////////////////////////
///
///    UPDATING PROCEDURES

/////////   1-   UPDATE TIME
/////////

void ChLink::UpdateTime(double time) {
    ChTime = time;
}

/////////
/////////   COMPLETE UPDATE
/////////
/////////

void ChLink::Update(double time, bool update_assets) {
    // 1 -
    UpdateTime(time);
}

void ChLink::Update(bool update_assets) {
    Update(ChTime, update_assets);  // use the same time
}

/////////
///////// FILE I/O
/////////


void ChLink::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkBase::ArchiveOUT(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLink::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLinkBase::ArchiveIN(marchive);

    // deserialize all member data:
}




}  // END_OF_NAMESPACE____
