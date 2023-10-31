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

#include "chrono/assets/ChVisualShapeSphere.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeSphere)

ChVisualShapeSphere::ChVisualShapeSphere() {
    SetMutable(false);
}

ChVisualShapeSphere::ChVisualShapeSphere(double radius) {
    gsphere.rad = radius;
    SetMutable(false);
}

ChVisualShapeSphere::ChVisualShapeSphere(const geometry::ChSphere& sphere) : gsphere(sphere) {
    SetMutable(false);
}

void ChVisualShapeSphere::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeSphere>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gsphere);
}

void ChVisualShapeSphere::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShapeSphere>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gsphere);
}

}  // end namespace chrono
