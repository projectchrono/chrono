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

#include "chrono/collision/ChCollisionShapeCone.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeCone)
CH_UPCASTING(ChCollisionShapeCone, ChCollisionShape)

ChCollisionShapeCone::ChCollisionShapeCone() : ChCollisionShape(Type::CONE) {}

ChCollisionShapeCone::ChCollisionShapeCone(std::shared_ptr<ChContactMaterial> material, double radius, double height)
    : ChCollisionShape(Type::CONE, material) {
    gcone.r = radius;
    gcone.h = height;
}

ChCollisionShapeCone::ChCollisionShapeCone(std::shared_ptr<ChContactMaterial> material, const ChCone& cone)
    : ChCollisionShape(Type::CONE, material), gcone(cone) {}

void ChCollisionShapeCone::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeCone>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gcone);
}

void ChCollisionShapeCone::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeCone>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gcone);
}

}  // end namespace chrono
