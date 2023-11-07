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

#ifndef CH_COLLISION_SHAPE_CONVEX_HULL_H
#define CH_COLLISION_SHAPE_CONVEX_HULL_H

#include <vector>

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/core/ChVector.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision convex hull shape.
/// A convex hull is simply a point cloud that describe a convex polytope. Connectivity between the vertexes, as
/// faces/edges in triangle meshes is not necessary. Points are passed as a list which is then copied into the model.
class ChApi ChCollisionShapeConvexHull : public ChCollisionShape {
  public:
    ChCollisionShapeConvexHull();
    ChCollisionShapeConvexHull(std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
                               const std::vector<ChVector<>>& points         ///< list of hull points
    );

    ~ChCollisionShapeConvexHull() {}

    /// Access the list of vertices of thje convex hull.
    const std::vector<ChVector<>>& GetPoints() { return points; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    /// Create convex hull collsion shapes from the specified data file.
    /// All shapes are assigned the same contact material.
    static std::vector<std::shared_ptr<ChCollisionShapeConvexHull>> Read(std::shared_ptr<ChMaterialSurface> material,
                                                                         const std::string& filename);

  private:
    std::vector<ChVector<>> points;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
