//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <stdlib.h>
#include <algorithm>

#include "physics/ChIndexedNodes.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "core/ChLinearAlgebra.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChIndexedNodes> a_registration_ChIndexedNodes;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR INDEXED NODES CONTAINER

ChIndexedNodes::ChIndexedNodes() {
    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChIndexedNodes::~ChIndexedNodes() {
}

//////// FILE I/O

void ChIndexedNodes::ArchiveOUT(ChArchiveOut& marchive) {

    // class version number
    marchive.VersionWrite(1);

    // serialize parent class too
    ChPhysicsItem::ArchiveOUT(marchive);

    // stream out all member data
}

void ChIndexedNodes::ArchiveIN(ChArchiveIn& marchive) {

    // class version number
    int version = marchive.VersionRead();

    // deserialize parent class too
    ChPhysicsItem::ArchiveIN(marchive);

    // stream in all member data
}



}  // END_OF_NAMESPACE____

/////////////////////
