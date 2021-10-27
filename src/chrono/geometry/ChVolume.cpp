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
// Authors: Alessandro Tasora
// =============================================================================

#include <memory.h>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "chrono/geometry/ChVolume.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChVolume)  // NO! abstract class!

void ChVolume::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVolume>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    // marchive << CHNVP(closed);
}

void ChVolume::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVolume>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    // marchive >> CHNVP(closed);
}

}  // end namespace geometry
}  // end namespace chrono
