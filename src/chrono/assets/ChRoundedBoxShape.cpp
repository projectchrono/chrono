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

#include "chrono/assets/ChRoundedBoxShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedBoxShape)

ChRoundedBoxShape::ChRoundedBoxShape() {
    SetMutable(false);
}

ChRoundedBoxShape::ChRoundedBoxShape(double length_x, double length_y, double length_z, double radius) {
    gbox.SetLengths(ChVector<>(length_x, length_y, length_z));
    gbox.SetSphereRadius(radius);
    SetMutable(false);
}

ChRoundedBoxShape::ChRoundedBoxShape(const ChVector<>& lengths, double radius) {
    gbox.SetLengths(lengths);
    gbox.SetSphereRadius(radius);
    SetMutable(false);
}

ChRoundedBoxShape::ChRoundedBoxShape(const geometry::ChRoundedBox& box) : gbox(box) {
    SetMutable(false);
}

void ChRoundedBoxShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChRoundedBoxShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(gbox);
}

void ChRoundedBoxShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChRoundedBoxShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(gbox);
}

}  // end namespace chrono
