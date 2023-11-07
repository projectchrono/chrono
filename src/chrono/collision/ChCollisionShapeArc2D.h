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

#ifndef CH_COLLISION_SHAPE_ARC2D_H
#define CH_COLLISION_SHAPE_ARC2D_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChLineArc.h"

namespace chrono {

/// Collision 2D arc shape.
/// This is a low-level collsion shape that is automatically generated when adding a ChCollisionShapePath2D.
class ChApi ChCollisionShapeArc2D : public ChCollisionShape {
  public:
    ChCollisionShapeArc2D();
    ChCollisionShapeArc2D(std::shared_ptr<ChMaterialSurface> material, const geometry::ChLineArc& arc, double radius);

    ~ChCollisionShapeArc2D() {}

    /// Access the arc geometry.
    geometry::ChLineArc& GetGeometry() { return garc; }

    /// Get the arc thickness (the radius of a sweeping sphere).
    double GetSRadius() const { return radius; }

  private:
    geometry::ChLineArc garc;
    double radius;
};

}  // end namespace chrono

#endif
