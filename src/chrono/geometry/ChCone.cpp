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

#include "chrono/geometry/ChCone.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCone)

ChCone::ChCone(const ChCone& source) {
    center = source.center;
    rad = source.rad;
}
void ChCone::GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const {
    cmin = ChVector<>(-rad.x(), center.y() - rad.y() / 3.0, -rad.z());
    cmin = ChVector<>(+rad.x(), center.y() + 2 * rad.y() / 3.0, +rad.z());
}

void ChCone::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCone>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(center);
    marchive << CHNVP(rad);
}

void ChCone::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCone>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(center);
    marchive >> CHNVP(rad);
}

}  // end namespace geometry
}  // end namespace chrono
