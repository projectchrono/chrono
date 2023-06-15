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

ChGeometry::AABB ChTriangleMesh::GetBoundingBox(const ChMatrix33<>& rot) const {
    AABB bbox;
    for (int i = 0; i < getNumTriangles(); i++) {
        getTriangle(i).InflateBoundingBox(bbox, rot);
    }
    return bbox;
}

void ChTriangleMesh::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTriangleMesh>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
}

void ChTriangleMesh::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChTriangleMesh>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
}

}  // end namespace geometry
}  // end namespace chrono
