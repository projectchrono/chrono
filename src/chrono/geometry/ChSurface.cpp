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

#include <memory.h>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "chrono/geometry/ChSurface.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChSurface) // NO! abstract class

ChSurface::ChSurface(const ChSurface& source) {
    wireframe = source.wireframe;
}

void ChSurface::Normal(ChVector<>& dir, const double parU, const double parV) const {
    double bdf = 10e-9;
    double uA = 0, uB = 0;
    double vA = 0, vB = 0;

    if (parU > 0.5) {
        uB = parU;
        uA = parU - bdf;
    } else {
        uB = parU + bdf;
        uA = parU;
    }
    if (parV > 0.5) {
        vB = parV;
        vA = parV - bdf;
    } else {
        vB = parV + bdf;
        vA = parV;
    }
    ChVector<> V0, Vu, Vv;
    Evaluate(V0, uA, vA);
    Evaluate(Vu, uB, vA);
    Evaluate(Vv, uA, vB);
    dir = Vnorm(Vcross((Vu - V0), (Vv - V0)));
}

void ChSurface::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSurface>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    // marchive << CHNVP(closed);
}

void ChSurface::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChSurface>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    // marchive >> CHNVP(closed);
}

}  // end namespace geometry
}  // end namespace chrono
