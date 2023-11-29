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
    ChCollisionShapeEllipsoid(std::shared_ptr<ChMaterialSurface> material, double axis_x, double axis_y, double axis_z);
    ChCollisionShapeEllipsoid(std::shared_ptr<ChMaterialSurface> material, const ChVector<>& axes);
    ChCollisionShapeEllipsoid(std::shared_ptr<ChMaterialSurface> material, const geometry::ChEllipsoid& ellipsoid);

    ~ChCollisionShapeEllipsoid() {}

    // Access the ellipsoid geometry.
    geometry::ChEllipsoid& GetGeometry() { return gellipsoid; }

    /// Get the ellipsoid semiaxes.
    const ChVector<>& GetSemiaxes() const { return gellipsoid.GetSemiaxes(); }

    /// Get the ellipsoid axes.
    ChVector<> GetAxes() const { return gellipsoid.GetAxes(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChEllipsoid gellipsoid;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
