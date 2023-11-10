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

#include "chrono/collision/ChCollisionShapeEllipsoid.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeEllipsoid)
CH_UPCASTING(ChCollisionShapeEllipsoid, ChCollisionShape)

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid(): ChCollisionShape(Type::ELLIPSOID) {}

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid(std::shared_ptr<ChMaterialSurface> material,
                                                     double axis_x,
                                                     double axis_y,
                                                     double axis_z)
    : ChCollisionShape(Type::ELLIPSOID, material) {
    gellipsoid.rad = ChVector<>(axis_x / 2, axis_y / 2, axis_z / 2);
}

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid(std::shared_ptr<ChMaterialSurface> material,
                                                     const ChVector<>& axes)
    : ChCollisionShape(Type::ELLIPSOID, material) {
    gellipsoid.rad = axes / 2;
}

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid(std::shared_ptr<ChMaterialSurface> material,
                                                     const geometry::ChEllipsoid& ellipsoid)
    : ChCollisionShape(Type::ELLIPSOID, material), gellipsoid(ellipsoid) {}

void ChCollisionShapeEllipsoid::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeEllipsoid>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gellipsoid);
}

void ChCollisionShapeEllipsoid::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeEllipsoid>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gellipsoid);
}

}  // end namespace chrono
