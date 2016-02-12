//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Alessandro Tasora

#ifndef CHCONTACTSURFACE_H
#define CHCONTACTSURFACE_H

#include "chrono_fea/ChElementBase.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceBase.h"

namespace chrono {
namespace fea {

/// Base class for contact surfaces in FEA meshes.
/// Use children classes like ChContactSurfaceNodeCloud or ChContactSurfaceMesh
/// that implement practical functionalities.
/// The contact surface has a material of ChMaterialSurfaceBase type (DVI material
/// by default, but it can be also switched to a DEM material, etc, using Set).
class ChApiFea ChContactSurface {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChContactSurface);

  public:
    ChContactSurface(ChMesh* parentmesh = 0) {
        // default DVI material
        matsurface = std::make_shared<ChMaterialSurface>();
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
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceBase>& mnewsurf) { matsurface = mnewsurf; }

    /// Set the material surface for 'boundary contact'
    virtual std::shared_ptr<ChMaterialSurfaceBase>& GetMaterialSurfaceBase() { return matsurface; }

    /// Functions to interface this with ChPhysicsItem container
    virtual void SurfaceSyncCollisionModels() = 0;
    virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys) = 0;
    virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) = 0;

  protected:
    std::shared_ptr<ChMaterialSurfaceBase> matsurface;  ///< material for contacts

    ChMesh* mmesh;
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
