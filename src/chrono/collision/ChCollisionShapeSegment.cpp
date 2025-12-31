// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionShapeSegment.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeSegment)
CH_UPCASTING(ChCollisionShapeSegment, ChCollisionShape)

ChCollisionShapeSegment::ChCollisionShapeSegment()
    : ChCollisionShape(Type::SEGMENT), P1(nullptr), P2(nullptr), ownsP1(false), ownsP2(false), radius(0) {}

ChCollisionShapeSegment::ChCollisionShapeSegment(std::shared_ptr<ChContactMaterial> material,
                                                 const ChVector3d* point1,
                                                 const ChVector3d* point2,
                                                 bool owns_point1,
                                                 bool owns_point2,
                                                 double radius)
    : ChCollisionShape(Type::SEGMENT, material),
      P1(point1),
      P2(point2),
      ownsP1(owns_point1),
      ownsP2(owns_point2),
      radius(radius) {}

void ChCollisionShapeSegment::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChCollisionShapeSegment>();
    ChCollisionShape::ArchiveOut(archive_out);
    archive_out << CHNVP(radius);
}

void ChCollisionShapeSegment::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChCollisionShapeSegment>();
    ChCollisionShape::ArchiveIn(archive_in);
    archive_in >> CHNVP(radius);
}

}  // end namespace chrono
