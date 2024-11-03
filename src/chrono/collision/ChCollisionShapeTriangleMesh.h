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

#ifndef CH_COLLISION_SHAPE_TRIANGLE_MESH_H
#define CH_COLLISION_SHAPE_TRIANGLE_MESH_H

#include <vector>

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChTriangleMesh.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision mesh shape.
class ChApi ChCollisionShapeTriangleMesh : public ChCollisionShape {
  public:
    ChCollisionShapeTriangleMesh();
    ChCollisionShapeTriangleMesh(                     //
        std::shared_ptr<ChContactMaterial> material,  ///< surface contact material
        std::shared_ptr<ChTriangleMesh> mesh,         ///< mesh geometry
        bool is_static,                               ///< true if the model doesn't move. May improve performance.
        bool is_convex,                               ///< if true, a convex hull is used. May improve robustness.
        double radius = 0                             ///< outward sphere-swept layer (when supported)
    );

    ~ChCollisionShapeTriangleMesh() {}

    /// Access the mesh geometry.
    std::shared_ptr<ChTriangleMesh> GetMesh() { return trimesh; }

    /// Return true if the mesh is model does not move.
    bool IsStatic() const { return is_static; }

    /// Return true if the mesh is convex.
    bool IsConvex() const { return is_convex; }

    /// Return the mesh thickness as the radius of a sphere-swept mesh.
    double GetRadius() const { return radius; }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return trimesh->GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChTriangleMesh> trimesh;
    bool is_static;
    bool is_convex;
    double radius;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
