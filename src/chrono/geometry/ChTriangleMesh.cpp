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

void ChTriangleMesh::Transform(const ChVector3d displ, const ChQuaternion<> mquat) {
    this->Transform(displ, ChMatrix33<>(mquat));
}

ChAABB ChTriangleMesh::GetBoundingBox() const {
    ChAABB bbox;
    for (unsigned int i = 0; i < GetNumTriangles(); i++) {
        GetTriangle(i).InflateBoundingBox(bbox);
    }
    return bbox;
}

void ChTriangleMesh::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChTriangleMesh>();
    // serialize parent class
    ChGeometry::ArchiveOut(archive_out);
    // serialize all member data:
}

void ChTriangleMesh::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChTriangleMesh>();
    // deserialize parent class
    ChGeometry::ArchiveIn(archive_in);
    // stream in all member data:
}

}  // end namespace chrono
