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

#include "chrono/collision/ChCollisionShapeCapsule.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeCapsule)
CH_UPCASTING(ChCollisionShapeCapsule, ChCollisionShape)

ChCollisionShapeCapsule::ChCollisionShapeCapsule(): ChCollisionShape(Type::CAPSULE) {}

ChCollisionShapeCapsule::ChCollisionShapeCapsule(std::shared_ptr<ChMaterialSurface> material,
                                                 double radius,
                                                 double height)
    : ChCollisionShape(Type::CAPSULE, material) {
    gcapsule.r = radius;
    gcapsule.h = height;
}

ChCollisionShapeCapsule::ChCollisionShapeCapsule(std::shared_ptr<ChMaterialSurface> material,
                                                 const geometry::ChCapsule& cap)
    : ChCollisionShape(Type::CAPSULE, material), gcapsule(cap) {}

void ChCollisionShapeCapsule::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeCapsule>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gcapsule);
}

/// Method to allow de serialization of transient data from archives.
void ChCollisionShapeCapsule::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeCapsule>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gcapsule);
}

}  // end namespace chrono
