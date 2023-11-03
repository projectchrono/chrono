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

#ifndef CH_COLLISION_SHAPE_ROUNDED_CYLINDER_H
#define CH_COLLISION_SHAPE_ROUNDED_CYLINDER_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChCylinder.h"

namespace chrono {
namespace collision {

/// Collision rounded-cylinder shape.
/// When added to a collision model, the cylinder is defined with its axis along the Z direction of the shape frame.
class ChApi ChCollisionShapeRoundedCylinder : public ChCollisionShape {
  public:
    ChCollisionShapeRoundedCylinder();
    ChCollisionShapeRoundedCylinder(std::shared_ptr<ChMaterialSurface> material,
                                    double radius,
                                    double height,
                                    double sradius);
    ChCollisionShapeRoundedCylinder(std::shared_ptr<ChMaterialSurface> material,
                                    const geometry::ChCylinder& cyl,
                                    double sradius);

    ~ChCollisionShapeRoundedCylinder() {}

    // Access the cylinder geometry.
    geometry::ChCylinder& GetGeometry() { return gcylinder; }

    /// Get the cylinder radius.
    double GetRadius() const { return gcylinder.GetRadius(); }

    /// Get the cylinder height.
    double GetHeight() const { return gcylinder.GetHeight(); }

    /// Get the radius of the sweeping sphere.
    double GetSRadius() const { return radius; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChCylinder gcylinder;
    double radius;
};

}  // end namespace collision
}  // end namespace chrono

#endif
