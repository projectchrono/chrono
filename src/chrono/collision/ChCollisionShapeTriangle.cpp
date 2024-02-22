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

ChCollisionShapeTriangle::ChCollisionShapeTriangle(std::shared_ptr<ChContactMaterial> material,
                                                   const ChVector3d& p1,
                                                   const ChVector3d& p2,
                                                   const ChVector3d& p3)
    : ChCollisionShape(Type::TRIANGLE, material) {
    gtriangle.SetPoints(p1, p2, p3);
}

ChCollisionShapeTriangle::ChCollisionShapeTriangle(std::shared_ptr<ChContactMaterial> material,
                                                   const ChTriangle& triangle)
    : ChCollisionShape(Type::TRIANGLE, material), gtriangle(triangle) {}

void ChCollisionShapeTriangle::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeTriangle>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gtriangle);
}

void ChCollisionShapeTriangle::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeTriangle>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gtriangle);
}

}  // end namespace chrono
