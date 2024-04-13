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

#include "chrono/geometry/ChSurface.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChSurface) // NO! abstract class

ChSurface::ChSurface(const ChSurface& other) : ChGeometry(other) {
    wireframe = other.wireframe;
}

ChVector3d ChSurface::GetNormal(double parU, double parV) const {
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

    auto V0 = Evaluate(uA, vA);
    auto Vu = Evaluate(uB, vA);
    auto Vv = Evaluate(uA, vB);

    return Vnorm(Vcross((Vu - V0), (Vv - V0)));
}

void ChSurface::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSurface>();
    // serialize parent class
    ChGeometry::ArchiveOut(archive_out);
    // serialize all member data:
    // archive_out << CHNVP(closed);
}

void ChSurface::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSurface>();
    // deserialize parent class
    ChGeometry::ArchiveIn(archive_in);
    // stream in all member data:
    // archive_in >> CHNVP(closed);
}

}  // end namespace chrono
