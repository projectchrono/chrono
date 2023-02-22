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

#include "chrono/assets/ChCapsuleShape.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChCapsuleShape)

ChCapsuleShape::ChCapsuleShape() {
    SetMutable(false);
}

ChCapsuleShape::ChCapsuleShape(double radius, double length) {
    gcapsule.rad = radius;
    gcapsule.hlen = length / 2;
}

ChCapsuleShape::ChCapsuleShape(const geometry::ChCapsule& cap) : gcapsule(cap) {
    SetMutable(false);
}

void ChCapsuleShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCapsuleShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(gcapsule);
}

/// Method to allow de serialization of transient data from archives.
void ChCapsuleShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCapsuleShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcapsule);
}

}  // end namespace chrono
