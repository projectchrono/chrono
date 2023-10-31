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

#include "chrono/assets/ChVisualShapeCone.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeCone)

ChVisualShapeCone::ChVisualShapeCone() {
    SetMutable(false);
}

ChVisualShapeCone::ChVisualShapeCone(double radius, double height) {
    gcone.r = radius;
    gcone.h = height;
    SetMutable(false);
}

ChVisualShapeCone::ChVisualShapeCone(const geometry::ChCone& cone) : gcone(cone) {
    SetMutable(false);
}

void ChVisualShapeCone::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeCone>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gcone);
}

void ChVisualShapeCone::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChVisualShapeCone>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcone);
}

}  // end namespace chrono
