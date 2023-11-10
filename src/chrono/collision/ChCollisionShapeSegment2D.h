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

#ifndef CH_COLLISION_SHAPE_SEGMENT2D_H
#define CH_COLLISION_SHAPE_SEGMENT2D_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision 2D segment shape.
/// This is a low-level collsion shape that is automatically generated when adding a ChCollisionShapePath2D.
class ChApi ChCollisionShapeSegment2D : public ChCollisionShape {
  public:
    ChCollisionShapeSegment2D();
    ChCollisionShapeSegment2D(std::shared_ptr<ChMaterialSurface> material,
                              const geometry::ChLineSegment& segment,
                              double radius);

    ~ChCollisionShapeSegment2D() {}

    /// Access the segment geometry.
    geometry::ChLineSegment& GetGeometry() { return gsegment; }

    /// Get the segment thickness (the radius of a sweeping sphere).
    double GetSRadius() const { return radius; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChLineSegment gsegment;
    double radius;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
