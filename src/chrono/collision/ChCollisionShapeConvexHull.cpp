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

#include "chrono/collision/ChCollisionShapeConvexHull.h"

namespace chrono {
namespace collision {

ChCollisionShapeConvexHull::ChCollisionShapeConvexHull() : ChCollisionShape(Type::CONVEXHULL) {}

ChCollisionShapeConvexHull::ChCollisionShapeConvexHull(std::shared_ptr<ChMaterialSurface> material,
                                                       const std::vector<ChVector<>>& points)
    : ChCollisionShape(Type::CONVEXHULL, material) {
    this->points = points;
}

void ChCollisionShapeConvexHull::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeConvexHull>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(points);
}

void ChCollisionShapeConvexHull::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeConvexHull>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(points);
}

}  // end namespace collision
}  // end namespace chrono
