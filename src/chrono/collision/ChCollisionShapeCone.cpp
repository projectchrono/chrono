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

#include "chrono/collision/ChCollisionShapeCone.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeCone)
CH_UPCASTING(ChCollisionShapeCone, ChCollisionShape)

ChCollisionShapeCone::ChCollisionShapeCone(): ChCollisionShape(Type::CONE) {}

ChCollisionShapeCone::ChCollisionShapeCone(std::shared_ptr<ChMaterialSurface> material, double radius, double height)
    : ChCollisionShape(Type::CONE, material) {
    gcone.r = radius;
    gcone.h = height;
}

ChCollisionShapeCone::ChCollisionShapeCone(std::shared_ptr<ChMaterialSurface> material, const geometry::ChCone& cone)
    : ChCollisionShape(Type::CONE, material), gcone(cone) {}

void ChCollisionShapeCone::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeCone>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gcone);
}

void ChCollisionShapeCone::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeCone>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcone);
}

}  // end namespace chrono
