// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionShapeRoundedCylinder.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeRoundedCylinder)
CH_UPCASTING(ChCollisionShapeRoundedCylinder, ChCollisionShape)

ChCollisionShapeRoundedCylinder::ChCollisionShapeRoundedCylinder() : ChCollisionShape(Type::ROUNDEDCYL) {}

ChCollisionShapeRoundedCylinder::ChCollisionShapeRoundedCylinder(std::shared_ptr<ChMaterialSurface> material,
                                                                 double radius,
                                                                 double height,
                                                                 double sradius)
    : ChCollisionShape(Type::ROUNDEDCYL, material) {
    gcylinder.r = radius;
    gcylinder.h = height;
    gcylinder.sr = sradius;
}

ChCollisionShapeRoundedCylinder::ChCollisionShapeRoundedCylinder(std::shared_ptr<ChMaterialSurface> material,
                                                                 const geometry::ChRoundedCylinder& cyl)
    : ChCollisionShape(Type::ROUNDEDCYL, material), gcylinder(cyl) {}

void ChCollisionShapeRoundedCylinder::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeRoundedCylinder>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gcylinder);
}

void ChCollisionShapeRoundedCylinder::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeRoundedCylinder>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcylinder);
}

}  // end namespace chrono
