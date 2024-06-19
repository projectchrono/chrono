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

#include "chrono/collision/ChCollisionShapeBarrel.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeBarrel)
CH_UPCASTING(ChCollisionShapeBarrel, ChCollisionShape)

ChCollisionShapeBarrel::ChCollisionShapeBarrel() : ChCollisionShape(Type::BARREL) {}

ChCollisionShapeBarrel::ChCollisionShapeBarrel(std::shared_ptr<ChContactMaterial> material,
                                               double Y_low,
                                               double Y_high,
                                               double axis_vert,
                                               double axis_hor,
                                               double R_offset)
    : ChCollisionShape(Type::BARREL, material) {
    this->Y_low = Y_low;
    this->Y_high = Y_high;
    this->axis_vert = axis_vert;
    this->axis_hor = axis_hor;
    this->R_offset = R_offset;
}

void ChCollisionShapeBarrel::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeBarrel>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(Y_low);
    archive_out << CHNVP(Y_high);
    archive_out << CHNVP(axis_vert);
    archive_out << CHNVP(axis_hor);
    archive_out << CHNVP(R_offset);
}

void ChCollisionShapeBarrel::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeBarrel>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(Y_low);
    archive_in >> CHNVP(Y_high);
    archive_in >> CHNVP(axis_vert);
    archive_in >> CHNVP(axis_hor);
    archive_in >> CHNVP(R_offset);
}

}  // end namespace chrono
