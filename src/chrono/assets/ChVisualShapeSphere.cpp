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

#include "chrono/assets/ChVisualShapeSphere.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeSphere)

ChVisualShapeSphere::ChVisualShapeSphere() {
    SetMutable(false);
}

ChVisualShapeSphere::ChVisualShapeSphere(double radius) {
    gsphere.rad = radius;
    SetMutable(false);
}

ChVisualShapeSphere::ChVisualShapeSphere(const ChSphere& sphere) : gsphere(sphere) {
    SetMutable(false);
}

void ChVisualShapeSphere::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeSphere>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gsphere);
}

void ChVisualShapeSphere::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeSphere>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gsphere);
}

}  // end namespace chrono
