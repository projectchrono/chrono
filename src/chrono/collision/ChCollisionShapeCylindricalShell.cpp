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

#include "chrono/collision/ChCollisionShapeCylindricalShell.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeCylindricalShell)
CH_UPCASTING(ChCollisionShapeCylindricalShell, ChCollisionShape)

ChCollisionShapeCylindricalShell::ChCollisionShapeCylindricalShell() : ChCollisionShape(Type::CYLSHELL) {}

ChCollisionShapeCylindricalShell::ChCollisionShapeCylindricalShell(std::shared_ptr<ChMaterialSurface> material,
                                                                   double radius,
                                                                   double height)
    : ChCollisionShape(Type::CYLSHELL, material) {
    gcylinder.r = radius;
    gcylinder.h = height;
}

ChCollisionShapeCylindricalShell::ChCollisionShapeCylindricalShell(std::shared_ptr<ChMaterialSurface> material,
                                                                   const geometry::ChCylinder& cyl)
    : ChCollisionShape(Type::CYLINDER, material), gcylinder(cyl) {}

void ChCollisionShapeCylindricalShell::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeCylindricalShell>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gcylinder);
}

void ChCollisionShapeCylindricalShell::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeCylindricalShell>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcylinder);
}

}  // end namespace chrono
