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

#include "chrono/assets/ChSphereShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSphereShape)

ChSphereShape::ChSphereShape() {
    SetMutable(false);
}

ChSphereShape::ChSphereShape(double radius) {
    gsphere.rad = radius;
    SetMutable(false);
}

ChSphereShape::ChSphereShape(const geometry::ChSphere& sphere) : gsphere(sphere) {
    SetMutable(false);
}

void ChSphereShape::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSphereShape>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gsphere);
}

void ChSphereShape::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChSphereShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gsphere);
}

}  // end namespace chrono
