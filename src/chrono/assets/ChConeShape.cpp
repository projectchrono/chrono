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

#include "chrono/assets/ChConeShape.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChConeShape)

ChConeShape::ChConeShape() {
    SetMutable(false);
}

ChConeShape::ChConeShape(double radius, double height) {
    gcone.r = radius;
    gcone.h = height;
    SetMutable(false);
}

ChConeShape::ChConeShape(const geometry::ChCone& cone) : gcone(cone) {
    SetMutable(false);
}

void ChConeShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConeShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(gcone);
}

void ChConeShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChConeShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcone);
}

}  // end namespace chrono
