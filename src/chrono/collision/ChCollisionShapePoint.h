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

#ifndef CH_COLLISION_SHAPE_POINT_H
#define CH_COLLISION_SHAPE_POINT_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/core/ChVector3.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision point shape.
class ChApi ChCollisionShapePoint : public ChCollisionShape {
  public:
    ChCollisionShapePoint();
    ChCollisionShapePoint(std::shared_ptr<ChContactMaterial> material, const ChVector3d& point, double radius);

    ~ChCollisionShapePoint() {}

    /// Access the point.
    const ChVector3d& GetPoint() const { return point; }

    /// Get the associated radius.
    double GetRadius() const { return radius; }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChVector3d point;
    double radius;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
