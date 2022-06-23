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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cstdio>

#include "chrono/geometry/ChCylinder.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCylinder)

ChCylinder::ChCylinder(const ChCylinder& source) {
    p1 = source.p1;
    p2 = source.p2;
    rad = source.rad;
}

void ChCylinder::GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const {
    //// TODO: re-check this
    ChVector<> dims = ChVector<>(rad, p2.y() - p1.y(), rad);
    ChVector<> trsfCenter = rot.transpose() * Baricenter();
    cmin = trsfCenter - dims;
    cmax = trsfCenter + dims;
}

void ChCylinder::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCylinder>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(p1);
    marchive << CHNVP(p2);
}

void ChCylinder::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCylinder>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(p1);
    marchive >> CHNVP(p2);
}

}  // end namespace geometry
}  // end namespace chrono
