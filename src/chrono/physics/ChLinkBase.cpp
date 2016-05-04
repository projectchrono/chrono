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

#include "physics/ChLinkBase.h"
#include "physics/ChGlobal.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChLinkBase> a_registration_ChLinkBase;

// BUILDERS
ChLinkBase::ChLinkBase() {
    broken = false;
    valid = true;
    disabled = false;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

// DESTROYER
ChLinkBase::~ChLinkBase() {
}

void ChLinkBase::Copy(ChLinkBase* source) {
    // first copy the parent class data...
    ChPhysicsItem::Copy(source);

    broken = source->broken;
    valid = source->valid;
    disabled = source->disabled;
}



void ChLinkBase::ArchiveOUT(ChArchiveOut& marchive) const
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP_OUT(disabled);
    marchive << CHNVP_OUT(valid);
    marchive << CHNVP_OUT(broken);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkBase::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    // int version =
    marchive.VersionRead();

    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP_IN(disabled);
    marchive >> CHNVP_IN(valid);
    marchive >> CHNVP_IN(broken);
}


}  // END_OF_NAMESPACE____
