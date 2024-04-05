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

#include "chrono/collision/ChCollisionShapeSegment2D.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeSegment2D)
CH_UPCASTING(ChCollisionShapeSegment2D, ChCollisionShape)

ChCollisionShapeSegment2D::ChCollisionShapeSegment2D() : ChCollisionShape(Type::SEGMENT2D) {}

ChCollisionShapeSegment2D::ChCollisionShapeSegment2D(std::shared_ptr<ChContactMaterial> material,
                                                     const ChLineSegment& segment,
                                                     double radius)
    : ChCollisionShape(Type::SEGMENT2D, material), gsegment(segment) {
    this->radius = radius;
}

void ChCollisionShapeSegment2D::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeSegment2D>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gsegment);
    archive_out << CHNVP(radius);
}

void ChCollisionShapeSegment2D::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeSegment2D>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gsegment);
    archive_in >> CHNVP(radius);
}

}  // end namespace chrono
