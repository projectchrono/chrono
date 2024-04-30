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

#ifndef CH_COLLISION_SHAPE_BOX_H
#define CH_COLLISION_SHAPE_BOX_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChBox.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision box shape.
class ChApi ChCollisionShapeBox : public ChCollisionShape {
  public:
    ChCollisionShapeBox();
    ChCollisionShapeBox(std::shared_ptr<ChContactMaterial> material, double length_x, double length_y, double length_z);
    ChCollisionShapeBox(std::shared_ptr<ChContactMaterial> material, const ChVector3d& lengths);
    ChCollisionShapeBox(std::shared_ptr<ChContactMaterial> material, const ChBox& box);

    ~ChCollisionShapeBox() {}

    /// Access the box geometry.
    ChBox& GetGeometry() { return gbox; }

    /// Get the box half-lengths.
    const ChVector3d& GetHalflengths() const { return gbox.GetHalflengths(); }

    /// Get the box dimensions.
    ChVector3d GetLengths() const { return gbox.GetLengths(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChBox gbox;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
