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

ChCollisionShapeRoundedBox::ChCollisionShapeRoundedBox(std::shared_ptr<ChContactMaterial> material,
                                                       double length_x,
                                                       double length_y,
                                                       double length_z,
                                                       double sradius)
    : ChCollisionShape(Type::ROUNDEDBOX, material) {
    gbox.SetLengths(ChVector3d(length_x, length_y, length_z));
    gbox.SetSphereRadius(sradius);
}

ChCollisionShapeRoundedBox::ChCollisionShapeRoundedBox(std::shared_ptr<ChContactMaterial> material,
                                                       const ChVector3d& lengths,
                                                       double sradius)
    : ChCollisionShape(Type::ROUNDEDBOX, material) {
    gbox.SetLengths(lengths);
    gbox.SetSphereRadius(sradius);
}

ChCollisionShapeRoundedBox::ChCollisionShapeRoundedBox(std::shared_ptr<ChContactMaterial> material,
                                                       const ChRoundedBox& box)
    : ChCollisionShape(Type::ROUNDEDBOX, material), gbox(box) {}

void ChCollisionShapeRoundedBox::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeRoundedBox>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gbox);
}

void ChCollisionShapeRoundedBox::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeRoundedBox>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gbox);
}

}  // end namespace chrono
