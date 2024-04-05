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

ChCollisionShapeBox::ChCollisionShapeBox() : ChCollisionShape(Type::BOX) {}

ChCollisionShapeBox::ChCollisionShapeBox(std::shared_ptr<ChContactMaterial> material,
                                         double length_x,
                                         double length_y,
                                         double length_z)
    : ChCollisionShape(Type::BOX, material) {
    gbox.SetLengths(ChVector3d(length_x, length_y, length_z));
}

ChCollisionShapeBox::ChCollisionShapeBox(std::shared_ptr<ChContactMaterial> material, const ChVector3d& lengths)
    : ChCollisionShape(Type::BOX, material) {
    gbox.SetLengths(lengths);
}

ChCollisionShapeBox::ChCollisionShapeBox(std::shared_ptr<ChContactMaterial> material, const ChBox& box)
    : ChCollisionShape(Type::BOX, material), gbox(box) {}

void ChCollisionShapeBox::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeBox>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gbox);
}

void ChCollisionShapeBox::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeBox>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gbox);
}

}  // end namespace chrono
