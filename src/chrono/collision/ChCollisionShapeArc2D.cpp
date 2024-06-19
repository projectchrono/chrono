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

#include "chrono/collision/ChCollisionShapeArc2D.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeArc2D)
CH_UPCASTING(ChCollisionShapeArc2D, ChCollisionShape)

ChCollisionShapeArc2D::ChCollisionShapeArc2D() : ChCollisionShape(Type::ARC2D) {}

ChCollisionShapeArc2D::ChCollisionShapeArc2D(std::shared_ptr<ChContactMaterial> material,
                                             const ChLineArc& arc,
                                             double radius)
    : ChCollisionShape(Type::ARC2D, material), garc(arc) {
    this->radius = radius;
}

void ChCollisionShapeArc2D::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeArc2D>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(garc);
    archive_out << CHNVP(radius);
}

void ChCollisionShapeArc2D::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeArc2D>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(garc);
    archive_in >> CHNVP(radius);
}

}  // end namespace chrono
