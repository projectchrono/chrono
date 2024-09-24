// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Base class for interfacing between a Chrono system and an FSI system
// =============================================================================

#ifndef CH_FSI_INTERFACE_H
#define CH_FSI_INTERFACE_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceSegmentSet.h"

#include "chrono_fsi/ChApiFsi.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for processing the interface between Chrono and FSI modules.
class CH_FSI_API ChFsiInterface {
  public:
    /// Description of a rigid body exposed to the FSI system.
    struct FsiBody {
        std::shared_ptr<ChBody> body;  ///< rigid body exposed to FSI system
        ChVector3d fsi_force;          ///< fluid force at body COM (expressed in absolute frame)
        ChVector3d fsi_torque;         ///< induced torque (expressed in absolute frame)
    };

    /// Description of an FEA mesh with 1-D segments exposed to the FSI system.
    struct FsiMesh1D {
        unsigned int GetNumElements() const;
        unsigned int GetNumNodes() const;

      std::shared_ptr<fea::ChContactSurfaceSegmentSet> contact_surface;  ///< FEA contact segments
        std::map<std::shared_ptr<fea::ChNodeFEAxyz>, int> ptr2ind_map;     ///< pointer-based to index-based mapping
        std::map<int, std::shared_ptr<fea::ChNodeFEAxyz>> ind2ptr_map;     ///< index-based to pointer-based mapping
    };

    /// Description of an FEA mesh with 2-D faces exposed to the FSI system.
    struct FsiMesh2D {
        unsigned int GetNumElements() const;
        unsigned int GetNumNodes() const;

        std::shared_ptr<fea::ChContactSurfaceMesh> contact_surface;     ///< FEA trimesh skin
        std::map<std::shared_ptr<fea::ChNodeFEAxyz>, int> ptr2ind_map;  ///< pointer-based to index-based mapping
        std::map<int, std::shared_ptr<fea::ChNodeFEAxyz>> ind2ptr_map;  ///< index-based to pointer-based mapping
    };

    // ------------

    virtual ~ChFsiInterface();

    void SetVerbose(bool verbose) { m_verbose = verbose; }

    // ------------

    /// Add a rigid body.
    FsiBody& AddFsiBody(std::shared_ptr<ChBody> body);

    /// Add a flexible solid with segment set contact to the FSI system.
    FsiMesh1D& AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface);

    /// Add a flexible solid with surface mesh contact to the FSI system.
    FsiMesh2D& AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface);

    /// Initialize the FSI interface.
    virtual void Initialize();

    // ------------

    /// Get the number of FSI bodies.
    unsigned int GetNumBodies() const;

    /// Get the number of FSI 1-D meshes.
    unsigned int GetNumMeshes1D() const;

    /// Get the number of FSI 1-D mesh elements (segments).
    unsigned int GetNumElements1D() const;

    /// Get the number of FSI 1-D mesh nodes.
    unsigned int GetNumNodes1D() const;

    /// Get the number of FSI 2-D meshes.
    unsigned int GetNumMeshes2D() const;

    /// Get the number of FSI 2-D mesh elements (segments).
    unsigned int GetNumElements2D() const;

    /// Get the number of FSI 2-D mesh nodes.
    unsigned int GetNumNodes2D() const;

    // ------------
    
    /// Return the FSI applied force on the body with specified index.
    /// The force is applied at the body COM and is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyForce(size_t i) const;

    /// Return the FSI applied torque on the body with specified index.
    /// The torque is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyTorque(size_t i) const;

    // ------------

    /// Extract and load states of all FSI bodies.
    virtual void LoadBodyStates() = 0;

    /// Extract and load states of FEA mesh nodes.
    virtual void LoadMeshStates() = 0;

    /// Apply fluid forces and torques as external loads to the FSI bodies.
    virtual void ApplyBodyForces() = 0;

    /// Add fluid forces and torques as external forces to the nodes of FSI meshes.
    virtual void ApplyMeshForces() = 0;

  protected:
    ChFsiInterface(bool verbose);

    bool m_verbose;

    std::vector<FsiBody> m_fsi_bodies;      ///< rigid bodies exposed to the FSI system
    std::vector<FsiMesh1D> m_fsi_meshes1D;  ///< FEA meshes with 1-D segments exposed to the FSI system
    std::vector<FsiMesh2D> m_fsi_meshes2D;  ///< FEA meshes with 2-D faces exposed to the FSI system
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
