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

#include "chrono/assets/ChRoundedCylinderShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedCylinderShape)

ChRoundedCylinderShape::ChRoundedCylinderShape() {
    SetMutable(false);
}

ChRoundedCylinderShape::ChRoundedCylinderShape(double radius, double height, double sphere_radius) {
    groundedcyl.r = radius;
    groundedcyl.h = height;
    groundedcyl.sr = sphere_radius;
    SetMutable(false);
}

ChRoundedCylinderShape::ChRoundedCylinderShape(const geometry::ChRoundedCylinder& cyl) : groundedcyl(cyl) {
    SetMutable(false);
}

void ChRoundedCylinderShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChRoundedCylinderShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(groundedcyl);
}

void ChRoundedCylinderShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChRoundedCylinderShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(groundedcyl);
}

}  // end namespace chrono
