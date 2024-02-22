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
    gellipsoid.rad = ChVector3d(axis_x / 2, axis_y / 2, axis_z / 2);
    SetMutable(false);
}

ChVisualShapeEllipsoid::ChVisualShapeEllipsoid(const ChVector3d& axes) {
    gellipsoid.rad = axes / 2;
    SetMutable(false);
}

ChVisualShapeEllipsoid::ChVisualShapeEllipsoid(const ChEllipsoid& ellipsoid) : gellipsoid(ellipsoid) {
    SetMutable(false);
}

void ChVisualShapeEllipsoid::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeEllipsoid>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gellipsoid);
}

void ChVisualShapeEllipsoid::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeEllipsoid>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gellipsoid);
}

}  // end namespace chrono
