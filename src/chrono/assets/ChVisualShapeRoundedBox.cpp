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

#include "chrono/assets/ChVisualShapeRoundedBox.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeRoundedBox)

ChVisualShapeRoundedBox::ChVisualShapeRoundedBox() {
    SetMutable(false);
}

ChVisualShapeRoundedBox::ChVisualShapeRoundedBox(double length_x, double length_y, double length_z, double radius) {
    gbox.SetLengths(ChVector<>(length_x, length_y, length_z));
    gbox.SetSphereRadius(radius);
    SetMutable(false);
}

ChVisualShapeRoundedBox::ChVisualShapeRoundedBox(const ChVector<>& lengths, double radius) {
    gbox.SetLengths(lengths);
    gbox.SetSphereRadius(radius);
    SetMutable(false);
}

ChVisualShapeRoundedBox::ChVisualShapeRoundedBox(const geometry::ChRoundedBox& box) : gbox(box) {
    SetMutable(false);
}

void ChVisualShapeRoundedBox::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeRoundedBox>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gbox);
}

void ChVisualShapeRoundedBox::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShapeRoundedBox>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gbox);
}

}  // end namespace chrono
