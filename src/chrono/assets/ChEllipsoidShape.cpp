// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include "chrono/assets/ChEllipsoidShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChEllipsoidShape)

ChEllipsoidShape::ChEllipsoidShape() {
    SetMutable(false);
}

ChEllipsoidShape::ChEllipsoidShape(double x_length, double y_length, double z_length) {
    gellipsoid.rad = ChVector<>(x_length / 2, y_length / 2, z_length / 2);
}

ChEllipsoidShape::ChEllipsoidShape(const geometry::ChEllipsoid& ellipsoid) : gellipsoid(ellipsoid) {
    SetMutable(false);
}

void ChEllipsoidShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChEllipsoidShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(gellipsoid);
}

void ChEllipsoidShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChEllipsoidShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(gellipsoid);
}

}  // end namespace chrono
