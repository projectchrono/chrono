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

#include "chrono/assets/ChVisualShapeBox.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeBox)

ChVisualShapeBox::ChVisualShapeBox() {
    SetMutable(false);
}

ChVisualShapeBox::ChVisualShapeBox(double length_x, double length_y, double length_z) {
    gbox.SetLengths(ChVector3d(length_x, length_y, length_z));
    SetMutable(false);
}

ChVisualShapeBox::ChVisualShapeBox(const ChVector3d& lengths) {
    gbox.SetLengths(lengths);
    SetMutable(false);
}

ChVisualShapeBox::ChVisualShapeBox(const ChBox& box) : gbox(box) {
    SetMutable(false);
}

void ChVisualShapeBox::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeBox>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gbox);
}

void ChVisualShapeBox::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeBox>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gbox);
}

}  // end namespace chrono
