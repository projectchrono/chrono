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

#include <cmath>

#include "chrono/geometry/ChTriangleMesh.h"

namespace chrono {
namespace geometry {

void ChTriangleMesh::Transform(const ChVector<> displ, const ChQuaternion<> mquat) {
    this->Transform(displ, ChMatrix33<>(mquat));
}

void ChTriangleMesh::GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const {
    cmin = ChVector<>(+std::numeric_limits<double>::max());
    cmax = ChVector<>(-std::numeric_limits<double>::max());

    for (int i = 0; i < getNumTriangles(); i++) {
        getTriangle(i).InflateBoundingBox(cmin, cmax, rot);
    }
}

void ChTriangleMesh::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTriangleMesh>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
}

void ChTriangleMesh::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChTriangleMesh>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
}

}  // end namespace geometry
}  // end namespace chrono
