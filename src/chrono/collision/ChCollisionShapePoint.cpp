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

#include "chrono/collision/ChCollisionShapePoint.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapePoint)
CH_UPCASTING(ChCollisionShapePoint, ChCollisionShape)

ChCollisionShapePoint::ChCollisionShapePoint() : ChCollisionShape(Type::POINT), radius(0.01) {}

ChCollisionShapePoint::ChCollisionShapePoint(std::shared_ptr<ChContactMaterial> material,
                                             const ChVector3d& point,
                                             double radius)
    : ChCollisionShape(Type::POINT, material) {
    this->point = point;
    this->radius = radius;
}

void ChCollisionShapePoint::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapePoint>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(point);
    archive_out << CHNVP(radius);
}

void ChCollisionShapePoint::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapePoint>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(point);
    archive_in >> CHNVP(radius);
}

}  // end namespace chrono
