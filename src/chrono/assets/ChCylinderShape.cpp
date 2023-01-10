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

#include "chrono/assets/ChCylinderShape.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChCylinderShape)

ChCylinderShape::ChCylinderShape() {
    SetMutable(false);
}

ChCylinderShape::ChCylinderShape(double radius, double length) {
    gcylinder.rad = radius;
    gcylinder.p1 = ChVector<>(0, -length / 2, 0);
    gcylinder.p1 = ChVector<>(0, +length / 2, 0);
}

ChCylinderShape::ChCylinderShape(const geometry::ChCylinder& cyl) : gcylinder(cyl) {
    SetMutable(false);
}

void ChCylinderShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCylinderShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(gcylinder);
}

void ChCylinderShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCylinderShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcylinder);
}

}  // end namespace chrono
