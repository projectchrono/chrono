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

#include "chrono/geometry/ChSphere.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSphere)

ChSphere::ChSphere(const ChSphere& source) {
    rad = source.rad;
}

ChGeometry::AABB ChSphere::GetBoundingBox(const ChMatrix33<>& rot) const {
    return AABB(ChVector<>(-rad), ChVector<>(+rad));
}

double ChSphere::GetBoundingSphereRadius() const {
    return rad;
}

void ChSphere::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSphere>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(rad);
}

void ChSphere::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(rad);
}

}  // end namespace geometry
}  // end namespace chrono
