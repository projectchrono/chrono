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
    r = source.r;
    h = source.h;
}

ChGeometry::AABB ChCylinder::GetBoundingBox(const ChMatrix33<>& rot) const {
    return AABB(ChVector<>(-r, -r, -h / 2),  //
                ChVector<>(+r, +r, +h / 2));
}

void ChCylinder::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCylinder>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(r);
    marchive << CHNVP(h);
}

void ChCylinder::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCylinder>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(r);
    marchive >> CHNVP(h);
}

}  // end namespace geometry
}  // end namespace chrono
