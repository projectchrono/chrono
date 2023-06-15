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

#include "chrono/assets/ChModelFileShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChModelFileShape)

ChModelFileShape::ChModelFileShape() : filename("") {
    SetMutable(false);
}

ChModelFileShape::ChModelFileShape(const std::string& fname) : filename(fname) {
    SetMutable(false);
}

void ChModelFileShape::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChModelFileShape>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(filename);
}

void ChModelFileShape::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChModelFileShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(filename);
}

}  // end namespace chrono
