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

#include "chrono/assets/ChVisualShapeRoundedCylinder.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeRoundedCylinder)

ChVisualShapeRoundedCylinder::ChVisualShapeRoundedCylinder() {
    SetMutable(false);
}

ChVisualShapeRoundedCylinder::ChVisualShapeRoundedCylinder(double radius, double height, double sphere_radius) {
    groundedcyl.r = radius;
    groundedcyl.h = height;
    groundedcyl.sr = sphere_radius;
    SetMutable(false);
}

ChVisualShapeRoundedCylinder::ChVisualShapeRoundedCylinder(const geometry::ChRoundedCylinder& cyl) : groundedcyl(cyl) {
    SetMutable(false);
}

void ChVisualShapeRoundedCylinder::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeRoundedCylinder>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(groundedcyl);
}

void ChVisualShapeRoundedCylinder::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShapeRoundedCylinder>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(groundedcyl);
}

}  // end namespace chrono
