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
    gbox.SetLengths(ChVector<>(length_x, length_y, length_z));
    SetMutable(false);
}

ChVisualShapeBox::ChVisualShapeBox(const ChVector<>& lengths) {
    gbox.SetLengths(lengths);
    SetMutable(false);
}

ChVisualShapeBox::ChVisualShapeBox(const geometry::ChBox& box) : gbox(box) {
    SetMutable(false);
}

void ChVisualShapeBox::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeBox>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gbox);
}

void ChVisualShapeBox::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChVisualShapeBox>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gbox);
}

}  // end namespace chrono
