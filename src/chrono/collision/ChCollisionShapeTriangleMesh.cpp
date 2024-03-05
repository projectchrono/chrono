// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionShapeTriangleMesh.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeTriangleMesh)
CH_UPCASTING(ChCollisionShapeTriangleMesh, ChCollisionShape)

ChCollisionShapeTriangleMesh::ChCollisionShapeTriangleMesh() : ChCollisionShape(Type::TRIANGLEMESH), trimesh(nullptr){};

ChCollisionShapeTriangleMesh::ChCollisionShapeTriangleMesh(std::shared_ptr<ChContactMaterial> material,
                                                           std::shared_ptr<ChTriangleMesh> mesh,
                                                           bool is_static,
                                                           bool is_convex,
                                                           double radius)
    : ChCollisionShape(Type::TRIANGLEMESH, material), trimesh(mesh) {
    this->is_static = is_static;
    this->is_convex = is_convex;
    this->radius = radius;
}

void ChCollisionShapeTriangleMesh::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeTriangleMesh>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(trimesh);
    archive_out << CHNVP(is_static);
    archive_out << CHNVP(is_convex);
    archive_out << CHNVP(radius);
}

void ChCollisionShapeTriangleMesh::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeTriangleMesh>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(trimesh);
    archive_in >> CHNVP(is_static);
    archive_in >> CHNVP(is_convex);
    archive_in >> CHNVP(radius);
}

}  // end namespace chrono
