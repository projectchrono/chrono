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

#include "chrono/collision/ChCollisionShapeTriangle.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeTriangle)
CH_UPCASTING(ChCollisionShapeTriangle, ChCollisionShape)

ChCollisionShapeTriangle::ChCollisionShapeTriangle() : ChCollisionShape(Type::TRIANGLE) {}

ChCollisionShapeTriangle::ChCollisionShapeTriangle(std::shared_ptr<ChMaterialSurface> material,
                                                   const ChVector<>& p1,
                                                   const ChVector<>& p2,
                                                   const ChVector<>& p3)
    : ChCollisionShape(Type::TRIANGLE, material) {
    gtriangle.SetPoints(p1, p2, p3);
}

ChCollisionShapeTriangle::ChCollisionShapeTriangle(std::shared_ptr<ChMaterialSurface> material,
                                                   const geometry::ChTriangle& triangle)
    : ChCollisionShape(Type::TRIANGLE, material), gtriangle(triangle) {}

void ChCollisionShapeTriangle::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeTriangle>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gtriangle);
}

void ChCollisionShapeTriangle::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeTriangle>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gtriangle);
}

}  // end namespace chrono
