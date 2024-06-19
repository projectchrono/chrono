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

#include "chrono/assets/ChVisualShapeCone.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeCone)

ChVisualShapeCone::ChVisualShapeCone() {
    SetMutable(false);
}

ChVisualShapeCone::ChVisualShapeCone(double radius, double height) {
    gcone.r = radius;
    gcone.h = height;
    SetMutable(false);
}

ChVisualShapeCone::ChVisualShapeCone(const ChCone& cone) : gcone(cone) {
    SetMutable(false);
}

void ChVisualShapeCone::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeCone>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gcone);
}

void ChVisualShapeCone::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeCone>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gcone);
}

}  // end namespace chrono
