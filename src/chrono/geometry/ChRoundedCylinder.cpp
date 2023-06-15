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

ChRoundedCylinder::ChRoundedCylinder(double radius, double height, double sphere_radius)
    : r(radius), h(height), sr(sphere_radius) {}

ChRoundedCylinder::ChRoundedCylinder(const ChRoundedCylinder& source) {
    r = source.r;
    h = source.h;
    sr = source.sr;
}

void ChRoundedCylinder::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChRoundedCylinder>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(r);
    marchive << CHNVP(h);
    marchive << CHNVP(sr);
}

void ChRoundedCylinder::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChRoundedCylinder>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(r);
    marchive >> CHNVP(h);
    marchive >> CHNVP(sr);
}

}  // end namespace geometry
}  // end namespace chrono
