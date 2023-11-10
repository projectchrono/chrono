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

ChCollisionShapeArc2D::ChCollisionShapeArc2D(std::shared_ptr<ChMaterialSurface> material,
                                             const geometry::ChLineArc& arc,
                                             double radius)
    : ChCollisionShape(Type::ARC2D, material), garc(arc) {
    this->radius = radius;
}

void ChCollisionShapeArc2D::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeArc2D>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(garc);
    marchive << CHNVP(radius);
}

void ChCollisionShapeArc2D::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeArc2D>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(garc);
    marchive >> CHNVP(radius);
}

}  // end namespace chrono
