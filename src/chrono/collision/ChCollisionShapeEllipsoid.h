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

#ifndef CH_COLLISION_SHAPE_ELLIPSOID_H
#define CH_COLLISION_SHAPE_ELLIPSOID_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChEllipsoid.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision ellipsoid shape.
class ChApi ChCollisionShapeEllipsoid : public ChCollisionShape {
  public:
    ChCollisionShapeEllipsoid();
    ChCollisionShapeEllipsoid(std::shared_ptr<ChContactMaterial> material, double axis_x, double axis_y, double axis_z);
    ChCollisionShapeEllipsoid(std::shared_ptr<ChContactMaterial> material, const ChVector3d& axes);
    ChCollisionShapeEllipsoid(std::shared_ptr<ChContactMaterial> material, const ChEllipsoid& ellipsoid);

    ~ChCollisionShapeEllipsoid() {}

    // Access the ellipsoid geometry.
    ChEllipsoid& GetGeometry() { return gellipsoid; }

    /// Get the ellipsoid semiaxes.
    const ChVector3d& GetSemiaxes() const { return gellipsoid.GetSemiaxes(); }

    /// Get the ellipsoid axes.
    ChVector3d GetAxes() const { return gellipsoid.GetAxes(); }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return gellipsoid.GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChEllipsoid gellipsoid;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
