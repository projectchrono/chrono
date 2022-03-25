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

#include "chrono/geometry/ChEllipsoid.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChEllipsoid)

ChEllipsoid::ChEllipsoid(const ChEllipsoid& source) {
    rad = source.rad;
}

void ChEllipsoid::GetBoundingBox(double& xmin,
                                 double& xmax,
                                 double& ymin,
                                 double& ymax,
                                 double& zmin,
                                 double& zmax,
                                 ChMatrix33<>* Rot) const {
    xmin = -rad.x();
    xmax = +rad.x();
    ymin = -rad.y();
    ymax = +rad.y();
    zmin = -rad.z();
    zmax = +rad.z();
}

void ChEllipsoid::CovarianceMatrix(ChMatrix33<>& C) const {
    C.setZero();
}

void ChEllipsoid::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChEllipsoid>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(rad);
}

void ChEllipsoid::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChEllipsoid>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(rad);
}

}  // end namespace geometry
}  // end namespace chrono
