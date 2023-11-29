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

#include "chrono/assets/ChVisualShapeCapsule.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeCapsule)

ChVisualShapeCapsule::ChVisualShapeCapsule() {
    SetMutable(false);
}

ChVisualShapeCapsule::ChVisualShapeCapsule(double radius, double height) {
    gcapsule.r = radius;
    gcapsule.h = height;
    SetMutable(false);
}

ChVisualShapeCapsule::ChVisualShapeCapsule(const geometry::ChCapsule& cap) : gcapsule(cap) {
    SetMutable(false);
}

void ChVisualShapeCapsule::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeCapsule>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gcapsule);
}

/// Method to allow de serialization of transient data from archives.
void ChVisualShapeCapsule::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShapeCapsule>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcapsule);
}

}  // end namespace chrono
