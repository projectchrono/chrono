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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

//// RADU:
////  - is there a need for having the mesh in the constructor?

#ifndef CHCONTACTSURFACE_H
#define CHCONTACTSURFACE_H

#include "chrono/fea/ChElementBase.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/collision/ChCollisionSystem.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

/// Base class for contact surfaces in FEA meshes.
/// Actual collision geometry is provided by derived classes (ChContactSurfaceNodeCloud or ChContactSurfaceMesh).
class ChApi ChContactSurface {
  public:
    ChContactSurface(std::shared_ptr<ChContactMaterial> material, ChPhysicsItem* mesh = nullptr);

    virtual ~ChContactSurface() {}

    /// Get the owner physics item (e.g., an FEA mesh).
    ChPhysicsItem* GetPhysicsItem() { return m_physics_item; }

    /// Set the owner physics item (e.g., an FEA mesh).
    void SetPhysicsItem(ChPhysicsItem* physics_item) { m_physics_item = physics_item; }

    /// Get the current axis-aligned bounding box.
    virtual ChAABB GetAABB() const = 0;

    /// Disable self-collisions (default: enabled).
    /// Calling this function results in all associated collision models being placed in the same collision family and
    /// disabling collisions within that family.
    /// Note: this function must be called *before* adding collision shapes to this contact surface. 
    void DisableSelfCollisions(int family);

    /// Get the surface contact material.
    std::shared_ptr<ChContactMaterial>& GetMaterialSurface() { return m_material; }

    // Functions to interface this with ChPhysicsItem container
    virtual void SyncCollisionModels() const = 0;
    virtual void AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const = 0;
    virtual void RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const = 0;

  protected:
    std::shared_ptr<ChContactMaterial> m_material;  ///< contact material properties
    ChPhysicsItem* m_physics_item;                  ///< associated physics item (e.g., an FEA mesh)
    bool m_self_collide;                            ///< include self-collisions?
    int m_collision_family;                         ///< collision family (if no self-collisions)
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
