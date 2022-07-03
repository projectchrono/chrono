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

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChGeometry)  // NO! Abstract class!

void ChGeometry::GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const {
    cmin = ChVector<>(0);
    cmax = ChVector<>(0);
}

void ChGeometry::InflateBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const {
    ChVector<> amin;
    ChVector<> amax;
    GetBoundingBox(amin, amax, rot);
    cmin = ChVector<>(ChMin(cmin.x(), amin.x()), ChMin(cmin.y(), amin.y()), ChMin(cmin.z(), amin.z()));
    cmax = ChVector<>(ChMax(cmax.x(), amax.x()), ChMax(cmax.y(), amax.y()), ChMax(cmax.z(), amax.z()));
}

double ChGeometry::GetBoundingSphereRadius() const {
    ChVector<> amin;
    ChVector<> amax;
    GetBoundingBox(amin, amax, ChMatrix33<>(1));
    return (amax - amin).Length() / 2;
}

void ChGeometry::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChGeometry>();
}

void ChGeometry::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChGeometry>();
}

}  // end namespace geometry
}  // end namespace chrono
