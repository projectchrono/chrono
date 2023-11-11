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

#include "chrono/collision/ChCollisionShapeBox.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeBox)
CH_UPCASTING(ChCollisionShapeBox, ChCollisionShape)

ChCollisionShapeBox::ChCollisionShapeBox(): ChCollisionShape(Type::BOX) {}

ChCollisionShapeBox::ChCollisionShapeBox(std::shared_ptr<ChMaterialSurface> material,
                                         double length_x,
                                         double length_y,
                                         double length_z)
    : ChCollisionShape(Type::BOX, material) {
    gbox.SetLengths(ChVector<>(length_x, length_y, length_z));
}

ChCollisionShapeBox::ChCollisionShapeBox(std::shared_ptr<ChMaterialSurface> material, const ChVector<>& lengths)
    : ChCollisionShape(Type::BOX, material) {
    gbox.SetLengths(lengths);
}

ChCollisionShapeBox::ChCollisionShapeBox(std::shared_ptr<ChMaterialSurface> material, const geometry::ChBox& box)
    : ChCollisionShape(Type::BOX, material), gbox(box) {}

void ChCollisionShapeBox::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeBox>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gbox);
}

void ChCollisionShapeBox::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeBox>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gbox);
}

}  // end namespace chrono
