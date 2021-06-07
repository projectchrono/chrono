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

#include "chrono/geometry/ChCapsule.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCapsule)

ChCapsule::ChCapsule(const ChCapsule& source) {
    center = source.center;
    rad = source.rad;
    hlen = source.hlen;
}

void ChCapsule::GetBoundingBox(double& xmin,
                               double& xmax,
                               double& ymin,
                               double& ymax,
                               double& zmin,
                               double& zmax,
                               ChMatrix33<>* Rot) const {
    ChVector<> trsfCenter = Rot ? Rot->transpose() * center : center;

    xmin = trsfCenter.x() - rad;
    xmax = trsfCenter.x() + rad;
    ymin = trsfCenter.y() - (rad + hlen);
    ymax = trsfCenter.y() + (rad + hlen);
    zmin = trsfCenter.z() - rad;
    zmax = trsfCenter.z() + rad;
}

void ChCapsule::CovarianceMatrix(ChMatrix33<>& C) const {
    C.setZero();
    C(0, 0) = center.x() * center.x();
    C(1, 1) = center.y() * center.y();
    C(2, 2) = center.z() * center.z();
}

void ChCapsule::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCapsule>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(center);
    marchive << CHNVP(rad);
    marchive << CHNVP(hlen);
}

void ChCapsule::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCapsule>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(center);
    marchive >> CHNVP(rad);
    marchive >> CHNVP(hlen);
}

}  // end namespace geometry
}  // end namespace chrono
