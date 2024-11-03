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

#ifndef CH_COLLISION_SHAPE_MESHTRIANGLE_H
#define CH_COLLISION_SHAPE_MESHTRIANGLE_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChTriangle.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision shape representing a triangle in a connected mesh.
/// Unlike a ChCollisionShapeTriangle, this collision shape also maintains information on neighboring vertices and edges
/// in the containing mesh. This allows a collision system to implement mesh collision without double-counting node and
/// edge interactions.
/// For efficiency, this object uses pointers to the vertices in the containing mesh.
class ChApi ChCollisionShapeMeshTriangle : public ChCollisionShape {
  public:
    ChCollisionShapeMeshTriangle();
    ChCollisionShapeMeshTriangle(                     //
        std::shared_ptr<ChContactMaterial> material,  ///< contact material
        const ChVector3d* V1,                         ///< vertex1 coords
        const ChVector3d* V2,                         ///< vertex2 coords
        const ChVector3d* V3,                         ///< vertex3 coords
        const ChVector3d* eP1,                        ///< neighboring vertex at edge1 if any
        const ChVector3d* eP2,                        ///< neighboring vertex at edge2 if any
        const ChVector3d* eP3,                        ///< neighboring vertex at edge3 if any
        bool ownsV1,                                  ///< vertex1 owned by this triangle (otherwise, owned by neighbor)
        bool ownsV2,                                  ///< vertex2 owned by this triangle (otherwise, owned by neighbor)
        bool ownsV3,                                  ///< vertex3 owned by this triangle (otherwise, owned by neighbor)
        bool ownsE1,                                  ///< edge1 owned by this triangle (otherwise, owned by neighbor)
        bool ownsE2,                                  ///< edge2 owned by this triangle (otherwise, owned by neighbor)
        bool ownsE3,                                  ///< edge3 owned by this triangle (otherwise, owned by neighbor)
        double sphere_radius                          ///< radius of swept sphere
    );
    ~ChCollisionShapeMeshTriangle() {}

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    const ChVector3d* V1;
    const ChVector3d* V2;
    const ChVector3d* V3;
    const ChVector3d* eP1;
    const ChVector3d* eP2;
    const ChVector3d* eP3;
    bool ownsV1;
    bool ownsV2;
    bool ownsV3;
    bool ownsE1;
    bool ownsE2;
    bool ownsE3;
    double sradius;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
