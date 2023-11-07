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

#ifndef CH_COLLISION_SHAPE_CAPSULE_H
#define CH_COLLISION_SHAPE_CAPSULE_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChCapsule.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision capsule shape.
/// When added to a collision model, the capsule is defined with its axis along the Z direction of the shape frame.
class ChApi ChCollisionShapeCapsule : public ChCollisionShape {
  public:
    ChCollisionShapeCapsule();
    ChCollisionShapeCapsule(std::shared_ptr<ChMaterialSurface> material, double radius, double height);
    ChCollisionShapeCapsule(std::shared_ptr<ChMaterialSurface> material, const geometry::ChCapsule& cap);

    ~ChCollisionShapeCapsule() {}

    /// Access the capsule geometry.
    geometry::ChCapsule& GetGeometry() { return gcapsule; }

    /// Get the capsule radius.
    double GetRadius() const { return gcapsule.GetRadius(); }

    /// Get the capsule height (length of cylindrical portion).
    double GetHeight() const { return gcapsule.GetHeight(); }

    /// Get the capsule total length.
    double GetLength() const { return gcapsule.GetLength(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChCapsule gcapsule;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
