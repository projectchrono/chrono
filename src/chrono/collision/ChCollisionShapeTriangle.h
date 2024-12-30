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

#ifndef CH_COLLISION_SHAPE_TRIANGLE_H
#define CH_COLLISION_SHAPE_TRIANGLE_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChTriangle.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision triangle shape.
class ChApi ChCollisionShapeTriangle : public ChCollisionShape {
  public:
    ChCollisionShapeTriangle();
    ChCollisionShapeTriangle(std::shared_ptr<ChContactMaterial> material,
                             const ChVector3d& p1,
                             const ChVector3d& p2,
                             const ChVector3d& p3);
    ChCollisionShapeTriangle(std::shared_ptr<ChContactMaterial> material, const ChTriangle& triangle);

    ~ChCollisionShapeTriangle() {}

    /// Access the triangle geometry.
    ChTriangle& GetGeometry() { return gtriangle; }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return gtriangle.GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChTriangle gtriangle;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
