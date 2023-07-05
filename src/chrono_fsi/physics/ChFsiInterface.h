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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Base class for processing the interface between Chrono and FSI modules
// =============================================================================
#ifndef CH_FSI_INTERFACE_H
#define CH_FSI_INTERFACE_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChFsiBase.h"

namespace chrono {

// Forward declarations
namespace fea {
class ChContactSurfaceMesh;
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF_3423;
}  // namespace fea

namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for processing the interface between Chrono and FSI modules.
class ChFsiInterface : public ChFsiBase {
  public:
    /// Constructor of the FSI interface class.
    ChFsiInterface(ChSystemFsi_impl& fsi,
                   std::shared_ptr<SimParams> params);

    /// Destructor of the FSI interface class.
    ~ChFsiInterface();

    /// Resize number of cable elements used in the flexible elements.
    void ResizeChronoCablesData(const std::vector<std::vector<int>>& CableElementsNodesSTDVector);

    /// Resize number of shell elements used in the flexible elements.
    void ResizeChronoShellsData(const std::vector<std::vector<int>>& ShellElementsNodesSTDVector);

    /// Copy rigid body states from ChSystem to FsiSystem, then to the GPU memory.
    void LoadBodyState_Chrono2Fsi(std::shared_ptr<FsiBodyStateD> fsiBodyStateD);

    /// Copy FEA mesh states from ChSystem to FsiSystem, then to the GPU memory.
    void LoadMeshState_Chrono2Fsi(std::shared_ptr<FsiMeshStateD> fsiMeshStateD);

    /// Read the surface-integrated pressure and viscous forces form the fluid/granular dynamics system,
    /// and add these forces and torques as external forces to the ChSystem rigid bodies.
    void ApplyBodyForce_Fsi2Chrono();

    /// Add forces and torques as external forces to the ChSystem flexible bodies.
    void ApplyMeshForce_Fsi2Chrono();

  private:
    struct FsiMesh {
        std::shared_ptr<fea::ChContactSurfaceMesh> contact_surface;     // FEA mesh skin
        std::map<std::shared_ptr<fea::ChNodeFEAxyz>, int> ptr2ind_map;  // pointer-based index-based mapping
        std::map<int, std::shared_ptr<fea::ChNodeFEAxyz>> ind2ptr_map;  // index-based to pointer-based mapping
        int num_bce;                                                    // number of BCE markers for this mesh
    };

    ChSystemFsi_impl& m_sysFSI;  ///< FSI system
    bool m_verbose;              ///< enable/disable m_verbose terminal output (default: true)

    std::vector<std::shared_ptr<ChBody>> m_fsi_bodies;  ///< rigid bodies exposed to the FSI system
    std::vector<FsiMesh> m_fsi_meshes;                  ///< FEA meshes exposed to the FSI system

    //// RADU - obsolete
    std::shared_ptr<fea::ChMesh> m_fsi_mesh;
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> m_fsi_nodes;  ///< FEA nodes available in FSI system

    friend class ChSystemFsi;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
