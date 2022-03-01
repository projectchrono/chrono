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

#include "chrono/assets/ChBarrelShape.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChBarrelShape)

ChBarrelShape::ChBarrelShape() {
    SetMutable(false);
}
ChBarrelShape::ChBarrelShape(double mHlow, double mHsup, double mRvert, double mRhor, double mRoffset)
    : Hlow(mHlow), Hsup(mHsup), Rvert(mRvert), Rhor(mRhor), Roffset(mRoffset) {
    SetMutable(false);
}

void ChBarrelShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChBarrelShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(Hlow);
	marchive << CHNVP(Hsup);
	marchive << CHNVP(Rvert);
	marchive << CHNVP(Rhor);
	marchive << CHNVP(Roffset);
}

/// Method to allow de serialization of transient data from archives.
void ChBarrelShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChBarrelShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(Hlow);
	marchive >> CHNVP(Hsup);
	marchive >> CHNVP(Rvert);
	marchive >> CHNVP(Rhor);
	marchive >> CHNVP(Roffset);
}

}  // end namespace chrono
