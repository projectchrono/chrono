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

#include "chrono/collision/ChCollisionShapeSegment2D.h"

namespace chrono {

ChCollisionShapeSegment2D::ChCollisionShapeSegment2D() : ChCollisionShape(Type::SEGMENT2D) {}

ChCollisionShapeSegment2D::ChCollisionShapeSegment2D(std::shared_ptr<ChMaterialSurface> material,
                                                     const geometry::ChLineSegment& segment,
                                                     double radius)
    : ChCollisionShape(Type::SEGMENT2D, material), gsegment(segment) {
    this->radius = radius;
}

}  // end namespace chrono
