// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COLLISION_SHAPE
#define CH_COLLISION_SHAPE

#include "chrono/core/ChApiCE.h"
#include "chrono/geometry/ChGeometry.h"
#include "chrono/physics/ChContactMaterial.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Class defining a collision shape.
class ChApi ChCollisionShape {
  public:
    /// Supported collision shape types.
    enum Type {
        SPHERE,
        ELLIPSOID,
        BOX,
        CYLINDER,
        CYLSHELL,
        CONVEXHULL,
        TRIANGLEMESH,  // triangle mesh (compound object)
        BARREL,        // Not supported in Chrono collision system
        POINT,
        SEGMENT,
        TRIANGLE,      // stand-alone collision triangle
        MESHTRIANGLE,  // triangle in a connected mesh
        CAPSULE,
        CONE,         // Not implemented in Bullet collision system
        ROUNDEDBOX,   // Not implemented in Bullet collision system
        ROUNDEDCYL,   // Not implemented in Bullet collision system
        TETRAHEDRON,  // Not implemented in Bullet collision system
        PATH2D,       // 2D path (compound object)
        SEGMENT2D,    // line segment (part of a 2D path)
        ARC2D,        // circlular arc (part of a 2D path)
        UNKNOWN_SHAPE
    };

    ChCollisionShape(Type type = Type::UNKNOWN_SHAPE);
    ChCollisionShape(Type type, std::shared_ptr<ChContactMaterial> material);
    virtual ~ChCollisionShape() {}

    Type GetType() const { return m_type; }

    std::shared_ptr<ChContactMaterial> GetMaterial() const { return m_material; }
    ChContactMethod GetContactMethod() const { return m_material->GetContactMethod(); }

    /// Get the shape bounding box.
    /// The default implementation returns an inverted AABB.
    virtual ChAABB GetBoundingBox() const { return ChAABB(); }

    /// Set this visualization shape as modifiable (default: false).
    /// Set to true to indicate that the asset may change and therefore requires updates (e.g. for a deformable
    /// triangular mesh). Note that this also includes changes in materials.
    /// A visualization system may take advantage of this setting to accelerate rendering.
    void SetMutable(bool val) { is_mutable = val; }

    /// Return true if the visualization shape is marked as modifiable.
    bool IsMutable() const { return is_mutable; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  protected:
    Type m_type;                                    ///< type of collision shape
    std::shared_ptr<ChContactMaterial> m_material;  ///< surface contact material
    bool is_mutable;                                ///< flag indicating whether the shape is rigid or deformable

    friend class ChCollisionModel;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
