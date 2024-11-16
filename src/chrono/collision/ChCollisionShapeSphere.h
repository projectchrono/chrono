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

#ifndef CH_COLLISION_SHAPE_SPHERE_H
#define CH_COLLISION_SHAPE_SPHERE_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChSphere.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision sphere shape.
class ChApi ChCollisionShapeSphere : public ChCollisionShape {
  public:
    ChCollisionShapeSphere();
    ChCollisionShapeSphere(std::shared_ptr<ChContactMaterial> material, double radius);
    ChCollisionShapeSphere(std::shared_ptr<ChContactMaterial> material, const ChSphere& sphere);

    ~ChCollisionShapeSphere() {}

    /// Access the sphere geometry.
    ChSphere& GetGeometry() { return gsphere; }

    /// Get the sphere radius.
    double GetRadius() const { return gsphere.GetRadius(); }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return gsphere.GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChSphere gsphere;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
