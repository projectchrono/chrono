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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHCONTACTSURFACE_H
#define CHCONTACTSURFACE_H

#include "chrono/fea/ChElementBase.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurface.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

/// Base class for contact surfaces in FEA meshes.
/// Use children classes like ChContactSurfaceNodeCloud or ChContactSurfaceMesh
/// that implement practical functionalities.
/// The contact surface has a material of ChMaterialSurface type (non-smooth contact material
/// by default, but it can be also switched to a smooth (penalty) contact material, using Set).
class ChApi ChContactSurface {

  public:
    ChContactSurface(ChMesh* parentmesh = 0) {
        // default non-smooth contact material
        matsurface = std::make_shared<ChMaterialSurfaceNSC>();
        mmesh = parentmesh;
    }

    virtual ~ChContactSurface() {}

    //
    // FUNCTIONS
    //

    /// Get owner mesh
    ChMesh* GetMesh() { return mmesh; }

    /// Set owner mesh
    void SetMesh(ChMesh* mm) { mmesh = mm; }

    /// Set the material surface for 'boundary contact'
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for 'boundary contact'
    std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() { return matsurface; }

    /// Functions to interface this with ChPhysicsItem container
    virtual void SurfaceSyncCollisionModels() = 0;
    virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys) = 0;
    virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) = 0;

  protected:
    std::shared_ptr<ChMaterialSurface> matsurface;  ///< material for contacts

    ChMesh* mmesh;
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
