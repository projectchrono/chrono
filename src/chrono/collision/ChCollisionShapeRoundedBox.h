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
#include "chrono/geometry/ChRoundedBox.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

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
    ChCollisionShapeRoundedBox(std::shared_ptr<ChMaterialSurface> material, const geometry::ChRoundedBox& box);

    ~ChCollisionShapeRoundedBox() {}

    /// Access the rounded box geometry.
    geometry::ChRoundedBox& GetGeometry() { return gbox; }

    /// Get the box half-lengths.
    const ChVector<>& GetHalflengths() const { return gbox.GetHalflengths(); }

    /// Get the box dimensions.
    ChVector<> GetLengths() const { return gbox.GetLengths(); }

    /// Get the radius of the sweeping sphere.
    double GetSRadius() const { return gbox.GetSphereRadius(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChRoundedBox gbox;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
