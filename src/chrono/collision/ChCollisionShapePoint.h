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
#include "chrono/core/ChVector.h"

namespace chrono {
namespace collision {

/// Collision point shape.
class ChApi ChCollisionShapePoint : public ChCollisionShape {
  public:
    ChCollisionShapePoint();
    ChCollisionShapePoint(std::shared_ptr<ChMaterialSurface> material, const ChVector<>& point, double radius);

    ~ChCollisionShapePoint() {}

    /// Access the point.
    const ChVector<>& GetPoint() const { return point; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    ChVector<> point;
    double radius;
};

}  // end namespace collision
}  // end namespace chrono

#endif
