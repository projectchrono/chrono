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

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid() : ChCollisionShape(Type::ELLIPSOID) {}

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid(std::shared_ptr<ChContactMaterial> material,
                                                     double axis_x,
                                                     double axis_y,
                                                     double axis_z)
    : ChCollisionShape(Type::ELLIPSOID, material) {
    gellipsoid.rad = ChVector3d(axis_x / 2, axis_y / 2, axis_z / 2);
}

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid(std::shared_ptr<ChContactMaterial> material,
                                                     const ChVector3d& axes)
    : ChCollisionShape(Type::ELLIPSOID, material) {
    gellipsoid.rad = axes / 2;
}

ChCollisionShapeEllipsoid::ChCollisionShapeEllipsoid(std::shared_ptr<ChContactMaterial> material,
                                                     const ChEllipsoid& ellipsoid)
    : ChCollisionShape(Type::ELLIPSOID, material), gellipsoid(ellipsoid) {}

void ChCollisionShapeEllipsoid::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeEllipsoid>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gellipsoid);
}

void ChCollisionShapeEllipsoid::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeEllipsoid>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gellipsoid);
}

}  // end namespace chrono
