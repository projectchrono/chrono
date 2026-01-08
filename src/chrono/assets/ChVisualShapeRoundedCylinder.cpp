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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/assets/ChVisualShapeRoundedCylinder.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeRoundedCylinder)

ChVisualShapeRoundedCylinder::ChVisualShapeRoundedCylinder() {}

ChVisualShapeRoundedCylinder::ChVisualShapeRoundedCylinder(double radius, double height, double sphere_radius) {
    groundedcyl.r = radius;
    groundedcyl.h = height;
    groundedcyl.sr = sphere_radius;
}

ChVisualShapeRoundedCylinder::ChVisualShapeRoundedCylinder(const ChRoundedCylinder& cyl) : groundedcyl(cyl) {}

void ChVisualShapeRoundedCylinder::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeRoundedCylinder>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(groundedcyl);
}

void ChVisualShapeRoundedCylinder::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeRoundedCylinder>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(groundedcyl);
}

}  // end namespace chrono
