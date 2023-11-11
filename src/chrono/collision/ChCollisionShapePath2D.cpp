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

#include "chrono/collision/ChCollisionShapePath2D.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapePath2D)
CH_UPCASTING(ChCollisionShapePath2D, ChCollisionShape)

ChCollisionShapePath2D::ChCollisionShapePath2D() : ChCollisionShape(Type::PATH2D) {}

ChCollisionShapePath2D::ChCollisionShapePath2D(std::shared_ptr<ChMaterialSurface> material,
                                               std::shared_ptr<geometry::ChLinePath> path,
                                               double radius)
    : ChCollisionShape(Type::PATH2D, material), gpath(path) {
    this->radius = radius;
}

void ChCollisionShapePath2D::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapePath2D>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gpath);
    marchive << CHNVP(radius);
}

void ChCollisionShapePath2D::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapePath2D>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gpath);
    marchive >> CHNVP(radius);
}

}  // end namespace chrono
