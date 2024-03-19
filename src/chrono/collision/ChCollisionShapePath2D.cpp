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

ChCollisionShapePath2D::ChCollisionShapePath2D(std::shared_ptr<ChContactMaterial> material,
                                               std::shared_ptr<ChLinePath> path,
                                               double radius)
    : ChCollisionShape(Type::PATH2D, material), gpath(path) {
    this->radius = radius;
}

void ChCollisionShapePath2D::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapePath2D>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gpath);
    archive_out << CHNVP(radius);
}

void ChCollisionShapePath2D::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapePath2D>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gpath);
    archive_in >> CHNVP(radius);
}

}  // end namespace chrono
