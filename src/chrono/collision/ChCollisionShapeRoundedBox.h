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

#ifndef CH_COLLISION_SHAPE_ROUNDED_BOX_H
#define CH_COLLISION_SHAPE_ROUNDED_BOX_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChBox.h"

namespace chrono {
namespace collision {

/// Collision rounded-box shape.
class ChApi ChCollisionShapeRoundedBox : public ChCollisionShape {
  public:
    ChCollisionShapeRoundedBox();
    ChCollisionShapeRoundedBox(std::shared_ptr<ChMaterialSurface> material,
                               double length_x,
                               double length_y,
                               double length_z,
                               double sradius);
    ChCollisionShapeRoundedBox(std::shared_ptr<ChMaterialSurface> material, const ChVector<>& lengths, double sradius);
    ChCollisionShapeRoundedBox(std::shared_ptr<ChMaterialSurface> material, const geometry::ChBox& box, double sradius);

    ~ChCollisionShapeRoundedBox() {}

    /// Access the box geometry.
    geometry::ChBox& GetGeometry() { return gbox; }

    /// Get the box half-lengths.
    const ChVector<>& GetHalflengths() const { return gbox.GetHalflengths(); }

    /// Get the box dimensions.
    ChVector<> GetLengths() const { return gbox.GetLengths(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChBox gbox;
    double radius;
};

}  // end namespace collision
}  // end namespace chrono

#endif
