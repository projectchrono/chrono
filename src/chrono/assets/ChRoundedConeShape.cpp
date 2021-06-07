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

#include "chrono/assets/ChRoundedConeShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedConeShape)

void ChRoundedConeShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChRoundedConeShape>();
    // serialize parent class
    ChVisualization::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(groundedcone);
}

void ChRoundedConeShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChRoundedConeShape>();
    // deserialize parent class
    ChVisualization::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(groundedcone);
}

}  // end namespace chrono
