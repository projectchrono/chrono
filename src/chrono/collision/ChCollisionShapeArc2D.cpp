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

#include "chrono/collision/ChCollisionShapeArc2D.h"

namespace chrono {
namespace collision {

ChCollisionShapeArc2D::ChCollisionShapeArc2D() : ChCollisionShape(Type::ARC2D) {}

ChCollisionShapeArc2D::ChCollisionShapeArc2D(std::shared_ptr<ChMaterialSurface> material,
                                             const geometry::ChLineArc& arc,
                                             double radius)
    : ChCollisionShape(Type::ARC2D, material), garc(arc) {
    this->radius = radius;
}

}  // end namespace collision
}  // end namespace chrono
