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

#include "chrono/assets/ChBoxShape.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChBoxShape)

ChBoxShape::ChBoxShape() {
    SetMutable(false);
}

ChBoxShape::ChBoxShape(double length_x, double length_y, double length_z) {
    gbox.SetLengths(ChVector<>(length_x, length_y, length_z));
    SetMutable(false);
}

ChBoxShape::ChBoxShape(const ChVector<>& lengths) {
    gbox.SetLengths(lengths);
    SetMutable(false);
}

ChBoxShape::ChBoxShape(const geometry::ChBox& box) : gbox(box) {
    SetMutable(false);
}

void ChBoxShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChBoxShape>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(gbox);
}

void ChBoxShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChBoxShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(gbox);
}

}  // end namespace chrono
