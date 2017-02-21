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

#include <cfloat>
#include <cmath>
#include <memory.h>
#include <cstdlib>
#include <iostream>

#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChObject.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChObj)  // NO! Abstract class!

ChObj::ChObj() : ChTime(0) {
    identifier = GetUniqueIntID();
}

ChObj::ChObj(const ChObj& other) {
    identifier = GetUniqueIntID();

    name = other.name;
    ChTime = other.ChTime;
}

void ChObj::ArchiveOUT(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChObj>();

    // stream out all member data
    marchive << CHNVP(name);
    marchive << CHNVP(identifier);
    marchive << CHNVP(ChTime);
}

void ChObj::ArchiveIN(ChArchiveIn& marchive) {
    int version = marchive.VersionRead<ChObj>();

    // stream out all member data
    marchive >> CHNVP(name);
    marchive >> CHNVP(identifier);
    marchive >> CHNVP(ChTime);
}

}  // end namespace chrono
