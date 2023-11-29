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

#ifndef CH_COLLISION_SHAPE_PATH2D_H
#define CH_COLLISION_SHAPE_PATH2D_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChLinePath.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision 2D path shape.
/// When added to a collision model, the path is defined in the XY plane of the shape frame.
/// This shape defines a 2D collision shape that will collide with another 2D line of the same type if aligned on the
/// same plane. This is useful for mechanisms that work on a plane, and that require more precise collision that is not
/// possible with current 3D shapes. For example, the line can contain concave or convex round fillets. 
/// Requirements:
/// - the line must be clockwise for inner material, (counterclockwise=hollow, material outside)
/// - the line must contain only ChLineSegment and ChLineArc sub-lines
/// - the sublines must follow in the proper order, with coincident corners, and must be closed.
class ChApi ChCollisionShapePath2D : public ChCollisionShape {
  public:
    ChCollisionShapePath2D();
    ChCollisionShapePath2D(std::shared_ptr<ChMaterialSurface> material,
                           std::shared_ptr<geometry::ChLinePath> path,
                           double radius = 0.001);

    ~ChCollisionShapePath2D() {}

    /// Access the path geometry.
    std::shared_ptr<geometry::ChLinePath> GetGeometry() { return gpath; }

    /// Get the path thickness (the radius of a sweeping sphere).
    double GetSRadius() const { return radius; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    std::shared_ptr<geometry::ChLinePath> gpath;
    double radius;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
