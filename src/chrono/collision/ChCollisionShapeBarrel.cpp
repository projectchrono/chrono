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

ChCollisionShapeBarrel::ChCollisionShapeBarrel(std::shared_ptr<ChMaterialSurface> material,
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

void ChCollisionShapeBarrel::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeBarrel>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(Y_low);
    marchive << CHNVP(Y_high);
    marchive << CHNVP(axis_vert);
    marchive << CHNVP(axis_hor);
    marchive << CHNVP(R_offset);
}

void ChCollisionShapeBarrel::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeBarrel>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(Y_low);
    marchive >> CHNVP(Y_high);
    marchive >> CHNVP(axis_vert);
    marchive >> CHNVP(axis_hor);
    marchive >> CHNVP(R_offset);
}

}  // end namespace chrono
