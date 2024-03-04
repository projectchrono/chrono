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

ChCollisionShapeSegment2D::ChCollisionShapeSegment2D(std::shared_ptr<ChMaterialSurface> material,
                                                     const geometry::ChLineSegment& segment,
                                                     double radius)
    : ChCollisionShape(Type::SEGMENT2D, material), gsegment(segment) {
    this->radius = radius;
}

void ChCollisionShapeSegment2D::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeSegment2D>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gsegment);
    marchive << CHNVP(radius);
}

void ChCollisionShapeSegment2D::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeSegment2D>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gsegment);
    marchive >> CHNVP(radius);
}

}  // end namespace chrono
