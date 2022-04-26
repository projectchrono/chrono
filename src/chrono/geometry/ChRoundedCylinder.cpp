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

#include "chrono/geometry/ChRoundedCylinder.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedCylinder)

ChRoundedCylinder::ChRoundedCylinder(const ChRoundedCylinder& source) {
    rad = source.rad;
    hlen = source.hlen;
    radsphere = source.radsphere;
}

void ChRoundedCylinder::CovarianceMatrix(ChMatrix33<>& C) const {
    C.setZero();
}

void ChRoundedCylinder::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChRoundedCylinder>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(rad);
    marchive << CHNVP(hlen);
    marchive << CHNVP(radsphere);
}

void ChRoundedCylinder::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChRoundedCylinder>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(rad);
    marchive >> CHNVP(hlen);
    marchive >> CHNVP(radsphere);
}

}  // end namespace geometry
}  // end namespace chrono
