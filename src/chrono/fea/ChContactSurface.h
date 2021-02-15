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
////  - SetMesh / GetMesh should be private (or eliminated).  Use friend classes

#ifndef CHCONTACTSURFACE_H
#define CHCONTACTSURFACE_H

#include "chrono/fea/ChElementBase.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_contact
/// @{

/// Base class for contact surfaces in FEA meshes.
/// Use children classes like ChContactSurfaceNodeCloud or ChContactSurfaceMesh that implement practical
/// functionalities.
class ChApi ChContactSurface {
  public:
    ChContactSurface(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh = nullptr);

    virtual ~ChContactSurface() {}

    /// Get owner mesh
    ChMesh* GetMesh() { return m_mesh; }

    /// Set owner mesh
    void SetMesh(ChMesh* mesh) { m_mesh = mesh; }

    /// Get the surface contact material
    std::shared_ptr<ChMaterialSurface>& GetMaterialSurface() { return m_material; }

    /// Functions to interface this with ChPhysicsItem container
    virtual void SurfaceSyncCollisionModels() = 0;
    virtual void SurfaceAddCollisionModelsToSystem(ChSystem* msys) = 0;
    virtual void SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) = 0;

  protected:
    std::shared_ptr<ChMaterialSurface> m_material;  ///< contact material properties
    ChMesh* m_mesh;                                 ///< associated mesh
};

/// @} fea_contact

}  // end namespace fea
}  // end namespace chrono

#endif
