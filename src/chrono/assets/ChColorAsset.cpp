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

#include "chrono/assets/ChColorAsset.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChColorAsset)

void ChColorAsset::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChColorAsset>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(color);
    marchive << CHNVP(fading);
}

/// Method to allow de-serialization of transient data from archives.
void ChColorAsset::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChColorAsset>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(color);
    marchive >> CHNVP(fading);
}

}  // end namespace chrono
