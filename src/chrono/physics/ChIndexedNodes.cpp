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

#include <stdlib.h>
#include <algorithm>

#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
ChClassRegisterABSTRACT<ChIndexedNodes> a_registration_ChIndexedNodes;

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

}  // end namespace chrono
