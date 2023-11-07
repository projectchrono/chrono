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

#ifndef CH_COLLISION_SHAPE_BARREL_H
#define CH_COLLISION_SHAPE_BARREL_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/core/ChVector.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision barrel shape.
/// When added to a collision model, the shape is aligned with its main axis in the Y direction of the shape frame.
/// The barrel shape is made by lathing an arc of an ellipse around the vertical Y axis. The center of the ellipse is on
/// Y=0 level, and it is offsetted by R_offset from the Y axis in radial direction. The two axes of the ellipse are
/// axis_vert (for the vertical direction, i.e. the axis parallel to Y) and axis_hor (for the axis that is perpendicular
/// to Y). Also, the solid is clamped with two discs on the top and the bottom, at levels Y_low and Y_high.
class ChApi ChCollisionShapeBarrel : public ChCollisionShape {
  public:
    ChCollisionShapeBarrel();
    ChCollisionShapeBarrel(std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
                           double Y_low,                                 ///< bottom level
                           double Y_high,                                ///< top level
                           double axis_vert,                             ///< ellipse axis in vertical direction
                           double axis_hor,                              ///< ellipse axis in horizontal direction
                           double R_offset                               ///< lateral offset (radius at top and bottom)
    );

    ~ChCollisionShapeBarrel() {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    double Y_low;
    double Y_high;
    double axis_vert;
    double axis_hor;
    double R_offset;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
