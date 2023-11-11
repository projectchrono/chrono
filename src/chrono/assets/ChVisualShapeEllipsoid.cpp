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

#include "chrono/assets/ChVisualShapeEllipsoid.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeEllipsoid)

ChVisualShapeEllipsoid::ChVisualShapeEllipsoid() {
    SetMutable(false);
}

ChVisualShapeEllipsoid::ChVisualShapeEllipsoid(double axis_x, double axis_y, double axis_z) {
    gellipsoid.rad = ChVector<>(axis_x / 2, axis_y / 2, axis_z / 2);
    SetMutable(false);
}

ChVisualShapeEllipsoid::ChVisualShapeEllipsoid(const ChVector<>& axes) {
    gellipsoid.rad = axes / 2;
    SetMutable(false);
}

ChVisualShapeEllipsoid::ChVisualShapeEllipsoid(const geometry::ChEllipsoid& ellipsoid) : gellipsoid(ellipsoid) {
    SetMutable(false);
}

void ChVisualShapeEllipsoid::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeEllipsoid>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gellipsoid);
}

void ChVisualShapeEllipsoid::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChVisualShapeEllipsoid>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gellipsoid);
}

}  // end namespace chrono
