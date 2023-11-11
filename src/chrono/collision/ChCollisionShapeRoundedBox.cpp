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

#include "chrono/collision/ChCollisionShapeRoundedBox.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeRoundedBox)
CH_UPCASTING(ChCollisionShapeRoundedBox, ChCollisionShape)

ChCollisionShapeRoundedBox::ChCollisionShapeRoundedBox() : ChCollisionShape(Type::ROUNDEDBOX) {}

ChCollisionShapeRoundedBox::ChCollisionShapeRoundedBox(std::shared_ptr<ChMaterialSurface> material,
                                                       double length_x,
                                                       double length_y,
                                                       double length_z,
                                                       double sradius)
    : ChCollisionShape(Type::ROUNDEDBOX, material) {
    gbox.SetLengths(ChVector<>(length_x, length_y, length_z));
    gbox.SetSphereRadius(sradius);
}

ChCollisionShapeRoundedBox::ChCollisionShapeRoundedBox(std::shared_ptr<ChMaterialSurface> material,
                                                       const ChVector<>& lengths,
                                                       double sradius)
    : ChCollisionShape(Type::ROUNDEDBOX, material) {
    gbox.SetLengths(lengths);
    gbox.SetSphereRadius(sradius);
}

ChCollisionShapeRoundedBox::ChCollisionShapeRoundedBox(std::shared_ptr<ChMaterialSurface> material,
                                                       const geometry::ChRoundedBox& box)
    : ChCollisionShape(Type::ROUNDEDBOX, material), gbox(box) {}

void ChCollisionShapeRoundedBox::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeRoundedBox>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gbox);
}

void ChCollisionShapeRoundedBox::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeRoundedBox>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gbox);
}

}  // end namespace chrono
