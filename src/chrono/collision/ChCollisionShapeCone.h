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

#ifndef CH_COLLISION_SHAPE_CONE_H
#define CH_COLLISION_SHAPE_CONE_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChCone.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision cone shape.
/// When added to a collision model, the cone is defined with its axis along the Z direction of the shape frame.
class ChApi ChCollisionShapeCone : public ChCollisionShape {
  public:
    ChCollisionShapeCone();
    ChCollisionShapeCone(std::shared_ptr<ChMaterialSurface> material, double radius, double height);
    ChCollisionShapeCone(std::shared_ptr<ChMaterialSurface> material, const geometry::ChCone& cone);

    ~ChCollisionShapeCone() {}

    /// Access the cone geometry.
    geometry::ChCone& GetGeometry() { return gcone; }

    /// Get the cone radius.
    double GetRadius() const { return gcone.GetRadius(); }

    /// Get the cone height.
    double GetHeight() const { return gcone.GetHeight(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChCone gcone;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
