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

#include "chrono/assets/ChVisualShapeCylinder.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeCylinder)

ChVisualShapeCylinder::ChVisualShapeCylinder() {
    SetMutable(false);
}

ChVisualShapeCylinder::ChVisualShapeCylinder(double radius, double height) {
    gcylinder.r = radius;
    gcylinder.h = height;
    SetMutable(false);
}

ChVisualShapeCylinder::ChVisualShapeCylinder(const ChCylinder& cyl) : gcylinder(cyl) {
    SetMutable(false);
}

void ChVisualShapeCylinder::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeCylinder>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gcylinder);
}

void ChVisualShapeCylinder::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeCylinder>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gcylinder);
}

}  // end namespace chrono
